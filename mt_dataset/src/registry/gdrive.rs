use std::fs::{self, OpenOptions};
use std::io::IsTerminal;
use std::path::Path;
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow};
use async_trait::async_trait;
use glob::Pattern;
use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};
use regex::Regex;
use reqwest::Client;
use reqwest::StatusCode;
use reqwest::header::{CONTENT_RANGE, CONTENT_TYPE, LOCATION, RANGE};
use serde::{Deserialize, Serialize};
use tokio::io::AsyncReadExt;
use tokio::sync::Mutex;

use futures::future::join_all;

use crate::model::bag_ref::BagRef;
use crate::registry::driver::{BagInfo, PushMeta, RegistryDriver, RemoteDescriptor};
use crate::registry::gdrive_auth;
use crate::storage::config::RegistryDownloadMode;

const DRIVE_FILES_API: &str = "https://www.googleapis.com/drive/v3/files";
const DRIVE_UPLOAD_API: &str = "https://www.googleapis.com/upload/drive/v3/files";
const RESUMABLE_CHUNK_MIN_BYTES: usize = 8 * 1024 * 1024;
const RESUMABLE_CHUNK_START_BYTES: usize = 32 * 1024 * 1024;
const RESUMABLE_CHUNK_MAX_BYTES: usize = 48 * 1024 * 1024;
const RESUMABLE_UPLOAD_RETRIES: usize = 4;

#[derive(Debug)]
struct CachedToken {
    header: String,  // "Bearer <access_token>"
    expires_at: u64, // unix seconds; cache evicted 60 s before this
}

#[derive(Debug, Clone)]
pub struct GDriveRegistry {
    pub name: String,
    folder_id: String,
    token_env: Option<String>,
    disable_ranged_download: bool,
    client: Client,
    token_cache: Arc<Mutex<Option<CachedToken>>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct MetaFile {
    bag: BagRef,
    original_bytes: u64,
    packed_bytes: u64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    tags: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    bundle_hash: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pointcloud: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    mcap_compression: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pushed_at: Option<u64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct PublicManifest {
    bag: BagRef,
    original_bytes: u64,
    packed_bytes: u64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    tags: Vec<String>,
    bundle_file_id: String,
    metadata_file_id: String,
    bundle_url: String,
    metadata_url: String,
}

#[derive(Debug, Deserialize)]
struct FilesListResponse {
    files: Vec<DriveFile>,
}

#[derive(Debug, Deserialize, Clone)]
struct DriveFile {
    id: String,
    name: String,
}

#[derive(Debug, Deserialize)]
struct DriveCreateResponse {
    id: String,
}

#[derive(Debug, Serialize)]
struct DriveCreateRequest {
    name: String,
    parents: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none", rename = "mimeType")]
    mime_type: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
struct ServiceAccountKey {
    client_email: String,
    private_key: String,
    #[serde(default = "default_token_uri")]
    token_uri: String,
}

#[derive(Debug, Serialize)]
struct ServiceAccountClaims<'a> {
    iss: &'a str,
    scope: &'a str,
    aud: &'a str,
    exp: u64,
    iat: u64,
}

#[derive(Debug, Deserialize)]
struct OAuthTokenResponse {
    access_token: String,
}

fn default_token_uri() -> String {
    "https://oauth2.googleapis.com/token".to_string()
}

/// Create a signed RS256 JWT from serializable claims and a PEM-encoded RSA private key.
fn sign_rs256_jwt<T: serde::Serialize>(claims: &T, private_key_pem: &[u8]) -> Result<String> {
    use base64::Engine as _;
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;

    let header = r#"{"alg":"RS256","typ":"JWT"}"#;
    let header_b64 = URL_SAFE_NO_PAD.encode(header);
    let claims_json = serde_json::to_string(claims)?;
    let claims_b64 = URL_SAFE_NO_PAD.encode(&claims_json);
    let signing_input = format!("{}.{}", header_b64, claims_b64);

    let der = pem_to_der(private_key_pem).context("failed to parse RSA private key PEM")?;
    let key_pair = ring::signature::RsaKeyPair::from_pkcs8(&der)
        .map_err(|e| anyhow!("invalid RSA private key (PKCS#8 DER required): {:?}", e))?;
    let rng = ring::rand::SystemRandom::new();
    let mut sig = vec![0u8; key_pair.public().modulus_len()];
    key_pair
        .sign(
            &ring::signature::RSA_PKCS1_SHA256,
            &rng,
            signing_input.as_bytes(),
            &mut sig,
        )
        .map_err(|e| anyhow!("RSA signing failed: {:?}", e))?;

    let sig_b64 = URL_SAFE_NO_PAD.encode(&sig);
    Ok(format!("{}.{}", signing_input, sig_b64))
}

/// Decode a PEM-encoded key (strips headers and base64-decodes the body).
fn pem_to_der(pem: &[u8]) -> Result<Vec<u8>> {
    use base64::Engine as _;
    use base64::engine::general_purpose::STANDARD;
    let pem_str = std::str::from_utf8(pem).context("PEM is not valid UTF-8")?;
    let b64: String = pem_str
        .lines()
        .filter(|l| !l.starts_with("-----"))
        .collect();
    STANDARD.decode(b64).context("PEM base64 decode failed")
}

impl GDriveRegistry {
    async fn invalidate_cached_auth_header(&self) {
        *self.token_cache.lock().await = None;
    }

    pub fn from_uri(
        name: &str,
        uri: &str,
        auth_env: Option<String>,
        download_mode: RegistryDownloadMode,
    ) -> Result<Self> {
        let folder_id = uri
            .strip_prefix("gdrive://")
            .ok_or_else(|| anyhow!("gdrive registry URI must start with gdrive://"))?
            .trim()
            .to_string();
        if folder_id.is_empty() {
            return Err(anyhow!(
                "gdrive URI must include a folder id: gdrive://<folder_id>"
            ));
        }
        Ok(Self {
            name: name.to_string(),
            folder_id,
            token_env: auth_env,
            disable_ranged_download: download_mode == RegistryDownloadMode::Streaming,
            client: Client::builder()
                .connect_timeout(Duration::from_secs(30))
                .timeout(Duration::from_secs(60 * 60))
                .build()?,
            token_cache: Arc::new(Mutex::new(None)),
        })
    }

    fn object_stem(&self, bag: &BagRef) -> String {
        bag.object_path().replace('/', "__")
    }

    fn bundle_name(&self, bag: &BagRef) -> String {
        format!("{}.bundle.marina.tar.gz", self.object_stem(bag))
    }

    fn metadata_name(&self, bag: &BagRef) -> String {
        format!("{}.metadata.json", self.object_stem(bag))
    }

    fn public_manifest_name(&self, bag: &BagRef) -> String {
        format!("{}.public.json", self.object_stem(bag))
    }

    async fn auth_header_optional(&self) -> Result<Option<String>> {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .context("system clock before unix epoch")?
            .as_secs();

        // Check in-memory cache first (with 60-second safety buffer)
        {
            let guard = self.token_cache.lock().await;
            if let Some(ref ct) = *guard {
                if ct.expires_at > now + 60 {
                    return Ok(Some(ct.header.clone()));
                }
            }
        }

        // Cache miss — resolve via existing paths
        let result = if let Some((token, exp)) = gdrive_auth::get_access_token(&self.name).await? {
            Some((format!("Bearer {}", token), exp))
        } else if let Some(var) = &self.token_env {
            if let Ok(secret) = std::env::var(var) {
                let (token, exp) = self
                    .service_account_access_token_from_secret(secret.trim())
                    .await?;
                Some((format!("Bearer {}", token), exp))
            } else {
                None
            }
        } else {
            None
        };

        if let Some((header, expires_at)) = result {
            *self.token_cache.lock().await = Some(CachedToken {
                header: header.clone(),
                expires_at,
            });
            Ok(Some(header))
        } else {
            Ok(None)
        }
    }

    async fn auth_header_required(&self) -> Result<String> {
        self.auth_header_optional().await?.ok_or_else(|| {
            anyhow!(
                "gdrive auth missing: run `marina registry auth {}` or set auth_env to a service-account JSON path",
                self.name
            )
        })
    }

    fn api_key_optional(&self) -> Option<String> {
        std::env::var("GOOGLE_DRIVE_API_KEY").ok()
    }

    async fn service_account_access_token_from_secret(
        &self,
        secret: &str,
    ) -> Result<(String, u64)> {
        if secret.is_empty() {
            return Err(anyhow!("empty gdrive auth_env value"));
        }
        let sa = self.try_load_service_account(secret)?.ok_or_else(|| {
            anyhow!("auth_env value is not a valid service-account JSON file path or JSON string")
        })?;
        self.service_account_access_token(&sa).await
    }

    fn try_load_service_account(&self, secret: &str) -> Result<Option<ServiceAccountKey>> {
        let path = Path::new(secret);
        if path.exists() && path.is_file() {
            let text = fs::read_to_string(path).with_context(|| {
                format!("failed reading service-account json {}", path.display())
            })?;
            return self.parse_service_account_json(&text);
        }

        if secret.trim_start().starts_with('{') {
            return self.parse_service_account_json(secret);
        }

        Ok(None)
    }

    fn parse_service_account_json(&self, text: &str) -> Result<Option<ServiceAccountKey>> {
        let parsed: serde_json::Value =
            serde_json::from_str(text).context("failed parsing gdrive auth json value")?;
        let typ = parsed.get("type").and_then(|v| v.as_str());
        if typ != Some("service_account") {
            return Ok(None);
        }
        let key: ServiceAccountKey = serde_json::from_value(parsed)
            .context("invalid service-account json fields for gdrive auth")?;
        Ok(Some(key))
    }

    async fn service_account_access_token(&self, key: &ServiceAccountKey) -> Result<(String, u64)> {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .context("system clock before unix epoch")?
            .as_secs();
        let expires_at = now + 3600;
        let claims = ServiceAccountClaims {
            iss: &key.client_email,
            scope: "https://www.googleapis.com/auth/drive",
            aud: &key.token_uri,
            exp: expires_at,
            iat: now,
        };
        let assertion = sign_rs256_jwt(&claims, key.private_key.as_bytes())
            .context("failed creating service-account JWT assertion")?;

        let token = self
            .client
            .post(&key.token_uri)
            .form(&[
                ("grant_type", "urn:ietf:params:oauth:grant-type:jwt-bearer"),
                ("assertion", assertion.as_str()),
            ])
            .send()
            .await
            .context("failed requesting OAuth token from Google")?
            .error_for_status()
            .context("Google OAuth token request failed")?
            .json::<OAuthTokenResponse>()
            .await
            .context("failed decoding Google OAuth token response")?;
        Ok((token.access_token, expires_at))
    }

    async fn query_files(&self, query: &str) -> Result<Vec<DriveFile>> {
        let auth = self.auth_header_optional().await?;
        let api_key = self.api_key_optional();

        if auth.is_none() && api_key.is_none() {
            let files = self.list_public_folder_files().await?;
            return Ok(filter_public_files(&files, query));
        }

        let mut req = self.client.get(DRIVE_FILES_API).query(&[
            ("q", query),
            ("fields", "files(id,name)"),
            ("supportsAllDrives", "true"),
            ("includeItemsFromAllDrives", "true"),
        ]);

        if let Some(auth) = auth {
            req = req.header("Authorization", auth);
        } else if let Some(key) = api_key.as_deref() {
            req = req.query(&[("key", key)]);
        }

        let resp = req
            .send()
            .await
            .context("failed querying Google Drive files")?
            .error_for_status()
            .context("Google Drive list query failed")?
            .json::<FilesListResponse>()
            .await
            .context("failed decoding Google Drive file list")?;
        Ok(resp.files)
    }

    async fn list_public_folder_files(&self) -> Result<Vec<DriveFile>> {
        let html = self
            .client
            .get(format!(
                "https://drive.google.com/embeddedfolderview?id={}#list",
                self.folder_id
            ))
            .send()
            .await
            .context("failed loading public gdrive folder page")?
            .error_for_status()
            .context("public gdrive folder page request failed")?
            .text()
            .await
            .context("failed reading public gdrive folder page")?;

        let mut files = Vec::new();
        let re = public_file_regex();
        for cap in re.captures_iter(&html) {
            let id = cap
                .get(1)
                .map(|m| m.as_str().to_string())
                .unwrap_or_default();
            let body = cap.get(2).map(|m| m.as_str()).unwrap_or_default();
            let name = strip_html_tags(body).trim().to_string();
            if !id.is_empty() && !name.is_empty() {
                files.push(DriveFile { id, name });
            }
        }

        if files.is_empty() {
            return Err(anyhow!(
                "public folder page exposed no files. Share the folder and files with anyone who has the link"
            ));
        }

        Ok(files)
    }

    async fn delete_by_name(&self, name: &str) -> Result<()> {
        let q = format!(
            "'{}' in parents and trashed = false and name = '{}'",
            self.folder_id, name
        );
        for file in self.query_files(&q).await? {
            let _ = self.delete_file(&file.id).await;
        }
        Ok(())
    }

    async fn delete_file(&self, id: &str) -> Result<()> {
        let auth = self.auth_header_required().await?;
        self.client
            .delete(format!("{}/{}", DRIVE_FILES_API, id))
            .header("Authorization", auth)
            .query(&[("supportsAllDrives", "true")])
            .send()
            .await
            .with_context(|| format!("failed deleting gdrive file {}", id))?
            .error_for_status()
            .with_context(|| format!("Google Drive delete failed for {}", id))?;
        Ok(())
    }

    async fn find_files_by_name_authenticated(&self, name: &str) -> Result<Vec<DriveFile>> {
        let auth = self.auth_header_required().await?;
        let q = format!(
            "'{}' in parents and trashed = false and name = '{}'",
            self.folder_id, name
        );
        let resp = self
            .client
            .get(DRIVE_FILES_API)
            .header("Authorization", auth)
            .query(&[
                ("q", q.as_str()),
                ("fields", "files(id,name)"),
                ("supportsAllDrives", "true"),
                ("includeItemsFromAllDrives", "true"),
            ])
            .send()
            .await
            .context("failed querying existing gdrive files by name")?
            .error_for_status()
            .context("Google Drive list by name failed")?
            .json::<FilesListResponse>()
            .await
            .context("failed decoding Google Drive file list by name")?;
        Ok(resp.files)
    }

    async fn create_drive_file(&self, name: &str, mime: &str) -> Result<String> {
        let mut existing = self.find_files_by_name_authenticated(name).await?;
        existing.sort_by(|a, b| a.id.cmp(&b.id));
        if let Some(first) = existing.first() {
            return Ok(first.id.clone());
        }

        let auth = self.auth_header_required().await?;
        let created = self
            .client
            .post(DRIVE_FILES_API)
            .header("Authorization", auth)
            .query(&[("fields", "id"), ("supportsAllDrives", "true")])
            .json(&DriveCreateRequest {
                name: name.to_string(),
                parents: vec![self.folder_id.clone()],
                mime_type: Some(mime.to_string()),
            })
            .send()
            .await
            .with_context(|| format!("failed creating Google Drive file {}", name))?
            .error_for_status()
            .with_context(|| format!("Google Drive create failed for {}", name))?
            .json::<DriveCreateResponse>()
            .await
            .with_context(|| format!("failed decoding create response for {}", name))?;
        Ok(created.id)
    }

    async fn upload_media(
        &self,
        file_id: &str,
        mime: &str,
        body: reqwest::Body,
        name: &str,
    ) -> Result<()> {
        let auth = self.auth_header_required().await?;
        self.client
            .patch(format!("{}/{}", DRIVE_UPLOAD_API, file_id))
            .header("Authorization", auth)
            .header("Content-Type", mime)
            .query(&[("uploadType", "media"), ("supportsAllDrives", "true")])
            .body(body)
            .send()
            .await
            .with_context(|| format!("failed uploading {} to Google Drive", name))?
            .error_for_status()
            .with_context(|| format!("Google Drive upload failed for {}", name))?;
        Ok(())
    }

    async fn start_resumable_upload_session(
        &self,
        file_id: &str,
        mime: &str,
        total_bytes: u64,
        name: &str,
    ) -> Result<String> {
        let auth = self.auth_header_required().await?;
        let resp = self
            .client
            .patch(format!("{}/{}", DRIVE_UPLOAD_API, file_id))
            .header("Authorization", auth)
            .header("X-Upload-Content-Type", mime)
            .header("X-Upload-Content-Length", total_bytes.to_string())
            .header("Content-Type", "application/json; charset=UTF-8")
            .query(&[("uploadType", "resumable"), ("supportsAllDrives", "true")])
            .body("{}")
            .send()
            .await
            .with_context(|| {
                format!(
                    "failed starting resumable Google Drive upload session for {}",
                    name
                )
            })?
            .error_for_status()
            .with_context(|| {
                format!(
                    "Google Drive resumable session creation failed for {}",
                    name
                )
            })?;

        let location = resp
            .headers()
            .get(LOCATION)
            .ok_or_else(|| {
                anyhow!(
                    "Google Drive resumable session missing Location header for {}",
                    name
                )
            })?
            .to_str()
            .context("invalid resumable session Location header")?
            .to_string();

        Ok(location)
    }

    async fn upload_file_resumable(
        &self,
        session_url: &str,
        mime: &str,
        path: &Path,
        total_bytes: u64,
        name: &str,
        pb: &ProgressBar,
    ) -> Result<()> {
        let auth = self.auth_header_required().await?;
        let mut file = tokio::fs::File::open(path)
            .await
            .with_context(|| format!("failed to open {}", path.display()))?;
        let mut chunk_size = RESUMABLE_CHUNK_START_BYTES;
        let mut chunk = vec![0u8; chunk_size];
        let mut sent = 0u64;

        while sent < total_bytes {
            if chunk.len() != chunk_size {
                chunk.resize(chunk_size, 0);
            }
            let n = file
                .read(&mut chunk)
                .await
                .with_context(|| format!("failed reading {}", path.display()))?;
            if n == 0 {
                break;
            }

            let end = sent + n as u64 - 1;
            let range = format!("bytes {}-{}/{}", sent, end, total_bytes);
            let payload = chunk[..n].to_vec();

            let mut attempt = 0usize;
            loop {
                let started = Instant::now();
                let resp = self
                    .client
                    .put(session_url)
                    .header("Authorization", &auth)
                    .header("Content-Type", mime)
                    .header("Content-Length", n.to_string())
                    .header("Content-Range", &range)
                    .body(payload.clone())
                    .send()
                    .await;

                match resp {
                    Ok(resp) => {
                        let status = resp.status();
                        if status == StatusCode::PERMANENT_REDIRECT || status.is_success() {
                            let elapsed = started.elapsed();
                            sent += n as u64;
                            pb.inc(n as u64);

                            if attempt == 0
                                && elapsed < Duration::from_secs(2)
                                && chunk_size < RESUMABLE_CHUNK_MAX_BYTES
                            {
                                chunk_size = (chunk_size * 2).min(RESUMABLE_CHUNK_MAX_BYTES);
                            } else if attempt > 0 && chunk_size > RESUMABLE_CHUNK_MIN_BYTES {
                                chunk_size = (chunk_size / 2).max(RESUMABLE_CHUNK_MIN_BYTES);
                            }
                            break;
                        }

                        if status.is_server_error() && attempt < RESUMABLE_UPLOAD_RETRIES {
                            attempt += 1;
                            if chunk_size > RESUMABLE_CHUNK_MIN_BYTES {
                                chunk_size = (chunk_size / 2).max(RESUMABLE_CHUNK_MIN_BYTES);
                            }
                            tokio::time::sleep(Duration::from_millis(250 * attempt as u64)).await;
                            continue;
                        }

                        let body = resp.text().await.unwrap_or_default();
                        return Err(anyhow!(
                            "Google Drive resumable upload failed for {} with status {}: {}",
                            name,
                            status,
                            body
                        ));
                    }
                    Err(err) => {
                        if attempt < RESUMABLE_UPLOAD_RETRIES {
                            attempt += 1;
                            if chunk_size > RESUMABLE_CHUNK_MIN_BYTES {
                                chunk_size = (chunk_size / 2).max(RESUMABLE_CHUNK_MIN_BYTES);
                            }
                            tokio::time::sleep(Duration::from_millis(250 * attempt as u64)).await;
                            continue;
                        }
                        return Err(err).with_context(|| {
                            format!("failed uploading chunk to Google Drive for {}", name)
                        });
                    }
                }
            }
        }

        if sent != total_bytes {
            return Err(anyhow!(
                "resumable upload finished early for {}: sent {} of {} bytes",
                name,
                sent,
                total_bytes
            ));
        }

        Ok(())
    }

    fn build_download_request(
        &self,
        url: &str,
        auth: Option<&str>,
        api_key: Option<&str>,
    ) -> reqwest::RequestBuilder {
        let mut req = self.client.get(url);
        if let Some(auth) = auth {
            req = req.header("Authorization", auth);
        } else if let Some(key) = api_key {
            req = req.query(&[("key", key)]);
        }
        req
    }

    fn parse_content_range_total(content_range: &str) -> Option<u64> {
        let (_range, total) = content_range.split_once('/')?;
        total.parse::<u64>().ok()
    }

    async fn download_with_adaptive_ranges(
        &self,
        url: &str,
        auth: Option<&str>,
        api_key: Option<&str>,
        out: &Path,
        title: &str,
    ) -> Result<()> {
        let mut auth_header = auth.map(|s| s.to_string());
        let probe = self
            .build_download_request(url, auth_header.as_deref(), api_key)
            .header(RANGE, "bytes=0-0")
            .send()
            .await
            .with_context(|| format!("failed probing ranged download {}", title))?;

        if probe.status() != StatusCode::PARTIAL_CONTENT {
            return self
                .stream_response_to_path(url, auth_header.as_deref(), api_key, out, title)
                .await;
        }

        let total = probe
            .headers()
            .get(CONTENT_RANGE)
            .and_then(|v| v.to_str().ok())
            .and_then(Self::parse_content_range_total)
            .unwrap_or(0);

        if total == 0 {
            return self
                .stream_response_to_path(url, auth_header.as_deref(), api_key, out, title)
                .await;
        }

        let pb = transfer_bar(total, title);
        let mut existing = fs::metadata(out).map(|m| m.len()).unwrap_or(0);
        if existing > total {
            existing = 0;
        }
        let mut file = if existing > 0 {
            OpenOptions::new()
                .append(true)
                .open(out)
                .with_context(|| format!("failed opening output file {}", out.display()))?
        } else {
            fs::File::create(out)
                .with_context(|| format!("failed creating output file {}", out.display()))?
        };

        use std::io::Write;
        let mut downloaded = existing;
        pb.set_position(downloaded.min(total));

        let mut chunk_size = RESUMABLE_CHUNK_START_BYTES as u64;
        while downloaded < total {
            let end = (downloaded + chunk_size - 1).min(total - 1);
            let range = format!("bytes={}-{}", downloaded, end);

            let mut attempt = 0usize;
            let mut unauthorized_retries = 0usize;
            loop {
                let started = Instant::now();
                let resp = self
                    .build_download_request(url, auth_header.as_deref(), api_key)
                    .header(RANGE, &range)
                    .send()
                    .await;

                match resp {
                    Ok(resp) => {
                        let status = resp.status();
                        if status == StatusCode::PARTIAL_CONTENT {
                            let bytes = resp.bytes().await.with_context(|| {
                                format!("failed reading ranged chunk {} for {}", range, title)
                            })?;
                            let received = bytes.len() as u64;
                            file.write_all(&bytes)?;
                            downloaded += received;
                            pb.set_position(downloaded.min(total));

                            let elapsed = started.elapsed();
                            if attempt == 0
                                && elapsed < Duration::from_secs(2)
                                && chunk_size < RESUMABLE_CHUNK_MAX_BYTES as u64
                            {
                                chunk_size = (chunk_size * 2).min(RESUMABLE_CHUNK_MAX_BYTES as u64);
                            } else if attempt > 0 && chunk_size > RESUMABLE_CHUNK_MIN_BYTES as u64 {
                                chunk_size = (chunk_size / 2).max(RESUMABLE_CHUNK_MIN_BYTES as u64);
                            }
                            break;
                        }

                        if status.is_server_error() && attempt < RESUMABLE_UPLOAD_RETRIES {
                            attempt += 1;
                            if chunk_size > RESUMABLE_CHUNK_MIN_BYTES as u64 {
                                chunk_size = (chunk_size / 2).max(RESUMABLE_CHUNK_MIN_BYTES as u64);
                            }
                            tokio::time::sleep(Duration::from_millis(250 * attempt as u64)).await;
                            continue;
                        }

                        if status == StatusCode::OK && downloaded == 0 {
                            pb.finish_and_clear();
                            return self
                                .stream_response_to_path(
                                    url,
                                    auth_header.as_deref(),
                                    api_key,
                                    out,
                                    title,
                                )
                                .await;
                        }

                        if status == StatusCode::UNAUTHORIZED
                            && api_key.is_none()
                            && auth_header.is_some()
                            && unauthorized_retries < 2
                        {
                            unauthorized_retries += 1;
                            self.invalidate_cached_auth_header().await;
                            let refreshed = self.auth_header_optional().await?;
                            if refreshed.is_some() {
                                auth_header = refreshed;
                                continue;
                            }
                        }

                        return Err(anyhow!(
                            "ranged download failed for {} with status {}",
                            title,
                            status
                        ));
                    }
                    Err(err) => {
                        if attempt < RESUMABLE_UPLOAD_RETRIES {
                            attempt += 1;
                            if chunk_size > RESUMABLE_CHUNK_MIN_BYTES as u64 {
                                chunk_size = (chunk_size / 2).max(RESUMABLE_CHUNK_MIN_BYTES as u64);
                            }
                            tokio::time::sleep(Duration::from_millis(250 * attempt as u64)).await;
                            continue;
                        }
                        return Err(err)
                            .with_context(|| format!("ranged download failed for {}", title));
                    }
                }
            }
        }

        pb.finish_and_clear();
        Ok(())
    }

    async fn upload_named_bytes(&self, name: &str, mime: &str, bytes: Vec<u8>) -> Result<String> {
        let file_id = self.create_drive_file(name, mime).await?;
        let total = bytes.len() as u64;
        let pb = transfer_bar(total, &format!("gdrive upload {}", name));
        self.upload_media(&file_id, mime, reqwest::Body::from(bytes), name)
            .await?;
        pb.set_position(total);
        pb.finish_and_clear();
        Ok(file_id)
    }

    async fn upload_named_file(&self, name: &str, mime: &str, path: &Path) -> Result<String> {
        let file_id = self.create_drive_file(name, mime).await?;
        let size = fs::metadata(path)
            .with_context(|| format!("failed to stat {}", path.display()))?
            .len();
        let pb = transfer_bar(size, &format!("gdrive upload {}", name));
        let session_url = self
            .start_resumable_upload_session(&file_id, mime, size, name)
            .await?;
        self.upload_file_resumable(&session_url, mime, path, size, name, &pb)
            .await?;
        pb.finish_and_clear();
        Ok(file_id)
    }

    async fn download_file_to_path(&self, id: &str, out: &Path, title: &str) -> Result<()> {
        let auth = self.auth_header_optional().await?;
        let api_key = self.api_key_optional();

        if self.disable_ranged_download {
            if auth.is_none() && api_key.is_none() {
                return self
                    .stream_response_to_path(&public_download_url(id), None, None, out, title)
                    .await;
            }
            let url = format!(
                "{}/{}?alt=media&supportsAllDrives=true",
                DRIVE_FILES_API, id
            );
            return self
                .stream_response_to_path(&url, auth.as_deref(), api_key.as_deref(), out, title)
                .await;
        }

        if auth.is_none() && api_key.is_none() {
            return self
                .download_with_adaptive_ranges(&public_download_url(id), None, None, out, title)
                .await;
        }

        let url = format!(
            "{}/{}?alt=media&supportsAllDrives=true",
            DRIVE_FILES_API, id
        );
        self.download_with_adaptive_ranges(&url, auth.as_deref(), api_key.as_deref(), out, title)
            .await
    }

    async fn download_public_url_to_path(&self, url: &str, out: &Path, title: &str) -> Result<()> {
        self.download_with_adaptive_ranges(url, None, None, out, title)
            .await
    }

    async fn stream_response_to_path(
        &self,
        url: &str,
        auth: Option<&str>,
        api_key: Option<&str>,
        out: &Path,
        title: &str,
    ) -> Result<()> {
        let mut auth_header = auth.map(|s| s.to_string());
        let mut unauthorized_retries = 0usize;
        let mut resp = loop {
            let resp = self
                .build_download_request(url, auth_header.as_deref(), api_key)
                .send()
                .await
                .with_context(|| format!("failed downloading {}", title))?;
            if resp.status() == StatusCode::UNAUTHORIZED
                && api_key.is_none()
                && auth_header.is_some()
                && unauthorized_retries < 2
            {
                unauthorized_retries += 1;
                self.invalidate_cached_auth_header().await;
                if let Some(refreshed) = self.auth_header_optional().await? {
                    auth_header = Some(refreshed);
                    continue;
                }
            }
            break resp;
        };

        let status = resp.status();
        if status != StatusCode::PARTIAL_CONTENT && response_content_type_is_html(&resp) {
            let first_chunk = resp
                .chunk()
                .await
                .with_context(|| format!("failed reading HTML error response for {}", title))?;
            if let Some(chunk) = first_chunk.as_deref() {
                if let Some(kind) = html_error_hint(chunk) {
                    if kind == DownloadContentHint::GoogleDriveQuotaExceeded {
                        return Err(anyhow!(
                            "Google Drive quota exceeded while downloading {}. \
                             Google Drive returned an HTML quota page. Retry after the quota resets.",
                            title
                        ));
                    }
                }
            }
            return Err(anyhow!(
                "downloaded content for {} returned Content-Type text/html, not archive bytes",
                title
            ));
        }

        let total = resp
            .error_for_status_ref()
            .with_context(|| format!("download failed: {}", title))?
            .content_length()
            .unwrap_or(0);
        let pb = if total > 0 {
            let pb = ProgressBar::new(total);
            if !std::io::stdout().is_terminal() {
                pb.set_draw_target(ProgressDrawTarget::hidden());
            }
            pb.set_style(
                ProgressStyle::with_template("{msg} [{bar:40.cyan/blue}] {bytes}/{total_bytes}")
                    .unwrap_or_else(|_| ProgressStyle::default_bar()),
            );
            pb.set_message(title.to_string());
            Some(pb)
        } else {
            Some(spinner(title))
        };

        let mut file = fs::File::create(out)
            .with_context(|| format!("failed creating output file {}", out.display()))?;
        use std::io::Write;
        while let Some(chunk) = resp.chunk().await? {
            file.write_all(&chunk)?;
            if let Some(pb) = &pb {
                pb.inc(chunk.len() as u64);
            }
        }
        if let Some(pb) = pb {
            pb.finish_and_clear();
        }
        Ok(())
    }

    async fn download_file_bytes(&self, id: &str) -> Result<Vec<u8>> {
        let auth = self.auth_header_optional().await?;
        let api_key = self.api_key_optional();

        let mut req = if auth.is_none() && api_key.is_none() {
            self.client.get(public_download_url(id))
        } else {
            self.client
                .get(format!("{}/{}", DRIVE_FILES_API, id))
                .query(&[("alt", "media"), ("supportsAllDrives", "true")])
        };

        if let Some(auth) = auth {
            req = req.header("Authorization", auth);
        } else if let Some(key) = api_key.as_deref() {
            req = req.query(&[("key", key)]);
        }

        let bytes = req
            .send()
            .await
            .with_context(|| format!("failed downloading {} from Google Drive", id))?
            .error_for_status()
            .with_context(|| format!("Google Drive download failed for {}", id))?
            .bytes()
            .await
            .context("failed reading Google Drive response bytes")?;
        Ok(bytes.to_vec())
    }

    async fn find_single_by_name(&self, name: &str) -> Result<DriveFile> {
        let q = format!(
            "'{}' in parents and trashed = false and name = '{}'",
            self.folder_id, name
        );
        let mut files = self.query_files(&q).await?;
        files
            .drain(..)
            .next()
            .ok_or_else(|| anyhow!("file '{}' not found in gdrive registry", name))
    }
}

#[async_trait]
impl RegistryDriver for GDriveRegistry {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    async fn push(
        &self,
        _registry_name: &str,
        bag: &BagRef,
        packed_file: &Path,
        meta: &PushMeta,
    ) -> Result<()> {
        let bundle_name = self.bundle_name(bag);
        let metadata_name = self.metadata_name(bag);
        let manifest_name = self.public_manifest_name(bag);

        let bundle_id = self
            .upload_named_file(&bundle_name, "application/gzip", packed_file)
            .await?;

        let metadata = MetaFile {
            bag: bag.clone().without_attachment(),
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            tags: bag.tags.clone(),
            bundle_hash: Some(meta.bundle_hash.clone()),
            pointcloud: Some(meta.pointcloud.clone()),
            mcap_compression: Some(meta.mcap_compression.clone()),
            pushed_at: Some(meta.pushed_at),
        };
        let metadata_bytes = serde_json::to_vec_pretty(&metadata)?;
        let metadata_id = self
            .upload_named_bytes(&metadata_name, "application/json", metadata_bytes)
            .await?;

        let manifest = PublicManifest {
            bag: bag.clone().without_attachment(),
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            tags: bag.tags.clone(),
            bundle_file_id: bundle_id.clone(),
            metadata_file_id: metadata_id.clone(),
            bundle_url: public_download_url(&bundle_id),
            metadata_url: public_download_url(&metadata_id),
        };
        let manifest_bytes = serde_json::to_vec_pretty(&manifest)?;
        self.upload_named_bytes(&manifest_name, "application/json", manifest_bytes)
            .await?;

        Ok(())
    }

    async fn bag_info(&self, bag: &BagRef) -> Result<Option<BagInfo>> {
        let metadata_name = self.metadata_name(bag);
        let file = match self.find_single_by_name(&metadata_name).await {
            Ok(f) => f,
            Err(_) => return Ok(None),
        };
        let bytes = self.download_file_bytes(&file.id).await?;
        let meta: MetaFile = serde_json::from_slice(&bytes)?;
        Ok(Some(BagInfo {
            bundle_hash: meta.bundle_hash,
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            pointcloud: meta.pointcloud,
            mcap_compression: meta.mcap_compression,
            pushed_at: meta.pushed_at,
        }))
    }

    async fn pull(&self, bag: &BagRef, out_packed_file: &Path) -> Result<RemoteDescriptor> {
        let auth = self.auth_header_optional().await?;
        let api_key = self.api_key_optional();

        if auth.is_none() && api_key.is_none() {
            let manifest_name = self.public_manifest_name(bag);
            let manifest_file = self.find_single_by_name(&manifest_name).await?;
            let manifest_bytes = self.download_file_bytes(&manifest_file.id).await?;
            let manifest: PublicManifest = serde_json::from_slice(&manifest_bytes)
                .context("failed parsing public manifest from gdrive")?;

            if let Some(parent) = out_packed_file.parent() {
                fs::create_dir_all(parent)?;
            }
            self.download_public_url_to_path(
                &manifest.bundle_url,
                out_packed_file,
                &format!("downloading {}", bag.without_attachment()),
            )
            .await?;

            return Ok(RemoteDescriptor {
                registry_name: self.name.clone(),
                bag: manifest.bag,
                original_bytes: manifest.original_bytes,
                packed_bytes: manifest.packed_bytes,
            });
        }

        let bundle_name = self.bundle_name(bag);
        let metadata_name = self.metadata_name(bag);

        let bundle = self.find_single_by_name(&bundle_name).await?;
        let metadata_file = self.find_single_by_name(&metadata_name).await?;

        if let Some(parent) = out_packed_file.parent() {
            fs::create_dir_all(parent)?;
        }
        self.download_file_to_path(
            &bundle.id,
            out_packed_file,
            &format!("{}", bag.without_attachment()),
        )
        .await?;

        let meta_bytes = self.download_file_bytes(&metadata_file.id).await?;
        let meta: MetaFile = serde_json::from_slice(&meta_bytes)
            .context("failed parsing metadata.json from gdrive")?;

        Ok(RemoteDescriptor {
            registry_name: self.name.clone(),
            bag: meta.bag,
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
        })
    }

    async fn list(&self, filter: &str) -> Result<Vec<BagRef>> {
        let pattern = Pattern::new(filter).or_else(|_| Pattern::new("*"))?;

        let manifest_query = format!(
            "'{}' in parents and trashed = false and name contains '.public.json'",
            self.folder_id
        );
        let manifest_files = self.query_files(&manifest_query).await?;

        let mut out = Vec::new();
        let manifest_downloads = join_all(
            manifest_files
                .iter()
                .map(|f| self.download_file_bytes(&f.id)),
        )
        .await;
        for bytes_result in manifest_downloads {
            let bytes = bytes_result?;
            let manifest: PublicManifest = match serde_json::from_slice(&bytes) {
                Ok(v) => v,
                Err(_) => continue,
            };
            let bag = manifest.bag.without_attachment();
            if pattern.matches(&bag.to_string()) {
                out.push(bag);
            }
        }

        if !out.is_empty() {
            out.sort_by_key(|b| b.to_string());
            out.dedup();
            return Ok(out);
        }

        let q = format!(
            "'{}' in parents and trashed = false and name contains '.metadata.json'",
            self.folder_id
        );
        let files = self.query_files(&q).await?;

        let meta_downloads = join_all(files.iter().map(|f| self.download_file_bytes(&f.id))).await;
        for (file, bytes_result) in files.iter().zip(meta_downloads) {
            let bytes = bytes_result?;
            let meta: MetaFile = serde_json::from_slice(&bytes)
                .with_context(|| format!("failed parsing metadata file {}", file.name))?;
            let bag = meta.bag.without_attachment();
            if pattern.matches(&bag.to_string()) {
                out.push(bag);
            }
        }

        out.sort_by_key(|b| b.to_string());
        out.dedup();
        Ok(out)
    }

    async fn list_with_info(&self, filter: &str) -> Result<Vec<(BagRef, Option<BagInfo>)>> {
        let pattern = Pattern::new(filter).or_else(|_| Pattern::new("*"))?;

        // Query metadata files once — each one contains both bag identity and encoding info.
        let q = format!(
            "'{}' in parents and trashed = false and name contains '.metadata.json'",
            self.folder_id
        );
        let files = self.query_files(&q).await?;

        let mut result: Vec<(BagRef, Option<BagInfo>)> = Vec::new();
        let downloads = join_all(files.iter().map(|f| self.download_file_bytes(&f.id))).await;
        for bytes_result in downloads {
            let bytes = match bytes_result {
                Ok(b) => b,
                Err(_) => continue,
            };
            let meta: MetaFile = match serde_json::from_slice(&bytes) {
                Ok(m) => m,
                Err(_) => continue,
            };
            let bag = meta.bag.without_attachment();
            if !pattern.matches(&bag.to_string()) {
                continue;
            }
            let info = BagInfo {
                bundle_hash: meta.bundle_hash,
                original_bytes: meta.original_bytes,
                packed_bytes: meta.packed_bytes,
                pointcloud: meta.pointcloud,
                mcap_compression: meta.mcap_compression,
                pushed_at: meta.pushed_at,
            };
            result.push((bag, Some(info)));
        }

        if !result.is_empty() {
            result.sort_by_key(|(b, _)| b.to_string());
            result.dedup_by_key(|(b, _)| b.to_string());
            return Ok(result);
        }

        // Fall back to public manifest files for old-style public registries.
        let manifest_query = format!(
            "'{}' in parents and trashed = false and name contains '.public.json'",
            self.folder_id
        );
        let manifest_files = self.query_files(&manifest_query).await?;
        let manifest_downloads = join_all(
            manifest_files
                .iter()
                .map(|f| self.download_file_bytes(&f.id)),
        )
        .await;
        for bytes_result in manifest_downloads {
            let bytes = match bytes_result {
                Ok(b) => b,
                Err(_) => continue,
            };
            let manifest: PublicManifest = match serde_json::from_slice(&bytes) {
                Ok(m) => m,
                Err(_) => continue,
            };
            let bag = manifest.bag.without_attachment();
            if pattern.matches(&bag.to_string()) {
                result.push((bag, None));
            }
        }

        result.sort_by_key(|(b, _)| b.to_string());
        result.dedup_by_key(|(b, _)| b.to_string());
        Ok(result)
    }

    async fn remove(&self, bag: &BagRef) -> Result<()> {
        self.delete_by_name(&self.bundle_name(bag)).await?;
        self.delete_by_name(&self.metadata_name(bag)).await?;
        self.delete_by_name(&self.public_manifest_name(bag)).await?;
        Ok(())
    }

    async fn check_connection(&self) -> Result<()> {
        if cfg!(test) {
            return Ok(());
        }

        let auth = self.auth_header_optional().await?;
        let mut req = self
            .client
            .get(DRIVE_FILES_API)
            .timeout(Duration::from_secs(5))
            .query(&[
                (
                    "q",
                    format!("'{}' in parents and trashed=false", self.folder_id),
                ),
                ("pageSize", "1".to_string()),
                ("fields", "files(id)".to_string()),
                ("supportsAllDrives", "true".to_string()),
                ("includeItemsFromAllDrives", "true".to_string()),
            ]);

        if let Some(auth) = auth {
            req = req.header("Authorization", auth);
        } else if let Some(key) = self.api_key_optional().as_deref() {
            req = req.query(&[("key", key.to_string())]);
        }

        req.send()
            .await
            .context("failed checking gdrive connectivity")?
            .error_for_status()
            .context("drive connectivity check returned error")?;
        Ok(())
    }

    async fn check_write_access(&self) -> Result<()> {
        let probe_name = format!(".marina_write_probe_{}_{}", self.name, now_secs());
        let file_id = self
            .create_drive_file(&probe_name, "application/octet-stream")
            .await
            .context("failed creating write probe file in Google Drive")?;

        let upload_result = self
            .upload_media(
                &file_id,
                "application/octet-stream",
                reqwest::Body::from(vec![0u8]),
                &probe_name,
            )
            .await;
        if let Err(err) = upload_result {
            let _ = self.delete_file(&file_id).await;
            return Err(err).context("failed writing probe content to Google Drive");
        }

        self.delete_file(&file_id)
            .await
            .context("failed deleting Google Drive write probe file")
    }
}

fn now_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

fn spinner(message: &str) -> ProgressBar {
    let pb = ProgressBar::new_spinner();
    if !std::io::stdout().is_terminal() {
        pb.set_draw_target(ProgressDrawTarget::hidden());
    }
    pb.set_style(
        ProgressStyle::with_template("{spinner} {msg}")
            .unwrap_or_else(|_| ProgressStyle::default_spinner())
            .tick_chars("|/-\\ "),
    );
    pb.set_message(message.to_string());
    pb.enable_steady_tick(std::time::Duration::from_millis(100));
    pb
}

fn transfer_bar(total: u64, message: &str) -> ProgressBar {
    let pb = if total > 0 {
        ProgressBar::new(total)
    } else {
        ProgressBar::new_spinner()
    };
    if !std::io::stdout().is_terminal() {
        pb.set_draw_target(ProgressDrawTarget::hidden());
    }
    pb.set_style(
        ProgressStyle::with_template("{msg} [{bar:40.green/blue}] {bytes}/{total_bytes} ({eta})")
            .unwrap_or_else(|_| ProgressStyle::default_bar()),
    );
    pb.set_message(message.to_string());
    pb
}

fn public_file_regex() -> &'static Regex {
    static RE: OnceLock<Regex> = OnceLock::new();
    RE.get_or_init(|| {
        Regex::new(
            r#"href=\"https://drive\.google\.com/file/d/([^/\"]+)/view[^\"]*\"[^>]*>(.*?)</a>"#,
        )
        .expect("valid public drive file regex")
    })
}

fn strip_html_tags(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut in_tag = false;
    for ch in s.chars() {
        match ch {
            '<' => in_tag = true,
            '>' => in_tag = false,
            _ if !in_tag => out.push(ch),
            _ => {}
        }
    }
    html_unescape_basic(&out)
}

fn html_unescape_basic(s: &str) -> String {
    s.replace("&amp;", "&")
        .replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&quot;", "\"")
        .replace("&#39;", "'")
}

fn filter_public_files(files: &[DriveFile], query: &str) -> Vec<DriveFile> {
    if let Some(name) = parse_query_name_eq(query) {
        return files
            .iter()
            .filter(|f| f.name == name)
            .cloned()
            .collect::<Vec<_>>();
    }
    if let Some(needle) = parse_query_name_contains(query) {
        return files
            .iter()
            .filter(|f| f.name.contains(&needle))
            .cloned()
            .collect::<Vec<_>>();
    }
    files.to_vec()
}

fn parse_query_name_eq(query: &str) -> Option<String> {
    let marker = "name = '";
    let start = query.find(marker)? + marker.len();
    let rest = &query[start..];
    let end = rest.find('\'')?;
    Some(rest[..end].to_string())
}

fn parse_query_name_contains(query: &str) -> Option<String> {
    let marker = "name contains '";
    let start = query.find(marker)? + marker.len();
    let rest = &query[start..];
    let end = rest.find('\'')?;
    Some(rest[..end].to_string())
}

fn public_download_url(id: &str) -> String {
    format!("https://drive.usercontent.google.com/download?id={id}&export=download&confirm=t")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DownloadContentHint {
    GoogleDriveQuotaExceeded,
    HtmlResponse,
}

fn html_error_hint(bytes: &[u8]) -> Option<DownloadContentHint> {
    let head = String::from_utf8_lossy(bytes).to_ascii_lowercase();
    let looks_like_html = head.contains("<!doctype html")
        || head.contains("<html")
        || head.contains("<head>")
        || head.contains("<body");
    if !looks_like_html {
        return None;
    }
    if head.contains("google drive") && head.contains("quota exceeded") {
        return Some(DownloadContentHint::GoogleDriveQuotaExceeded);
    }
    Some(DownloadContentHint::HtmlResponse)
}

fn response_content_type_is_html(resp: &reqwest::Response) -> bool {
    resp.headers()
        .get(CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .map(|ct| ct.to_ascii_lowercase().contains("text/html"))
        .unwrap_or(false)
}

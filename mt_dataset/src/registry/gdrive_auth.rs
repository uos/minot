use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

use crate::storage::config;

const BUNDLED_CLIENT_ID: &str =
    "675349975779-ad4jl33imq5fju18i1m8mn0qqicne87n.apps.googleusercontent.com";
const BUNDLED_CLIENT_SECRET: &str = "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";
use serde::{Deserialize, Serialize};

const TOKEN_ENDPOINT: &str = "https://oauth2.googleapis.com/token";
const DRIVE_SCOPE: &str = "https://www.googleapis.com/auth/drive";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StoredToken {
    pub client_id: String,
    pub client_secret: String,
    pub access_token: String,
    pub refresh_token: String,
    pub expires_at: u64,
}

impl StoredToken {
    fn is_expired(&self) -> bool {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        now >= self.expires_at.saturating_sub(60)
    }
}

fn token_path(registry_name: &str) -> Option<std::path::PathBuf> {
    config::config_dir()
        .ok()
        .map(|d| d.join("tokens").join(format!("{}.json", registry_name)))
}

fn load_stored_token(registry_name: &str) -> Option<StoredToken> {
    let text = std::fs::read_to_string(token_path(registry_name)?).ok()?;
    serde_json::from_str(&text).ok()
}

fn save_stored_token(registry_name: &str, token: &StoredToken) -> Result<()> {
    let path =
        token_path(registry_name).ok_or_else(|| anyhow!("could not determine config directory"))?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&path, serde_json::to_vec_pretty(token)?)?;
    Ok(())
}

/// Returns a valid access token and its expiry (unix seconds) for the registry,
/// refreshing if expired. Returns `None` if no stored token exists.
pub async fn get_access_token(registry_name: &str) -> Result<Option<(String, u64)>> {
    let mut token = match load_stored_token(registry_name) {
        Some(t) => t,
        None => return Ok(None),
    };

    if token.is_expired() {
        token = do_refresh(&token.client_id, &token.client_secret, &token.refresh_token).await?;
        save_stored_token(registry_name, &token)?;
    }

    Ok(Some((token.access_token, token.expires_at)))
}

#[derive(Debug, Clone)]
pub struct OAuthStatus {
    pub token_path: PathBuf,
    pub token_present: bool,
    pub token_valid: bool,
    pub expires_at: Option<u64>,
    pub refresh_error: Option<String>,
}

pub async fn oauth_status(registry_name: &str) -> Result<OAuthStatus> {
    let path =
        token_path(registry_name).ok_or_else(|| anyhow!("could not determine config directory"))?;

    let mut token = match load_stored_token(registry_name) {
        Some(token) => token,
        None => {
            return Ok(OAuthStatus {
                token_path: path,
                token_present: false,
                token_valid: false,
                expires_at: None,
                refresh_error: None,
            });
        }
    };

    if token.is_expired() {
        match do_refresh(&token.client_id, &token.client_secret, &token.refresh_token).await {
            Ok(refreshed) => {
                save_stored_token(registry_name, &refreshed)?;
                token = refreshed;
            }
            Err(err) => {
                return Ok(OAuthStatus {
                    token_path: path,
                    token_present: true,
                    token_valid: false,
                    expires_at: Some(token.expires_at),
                    refresh_error: Some(err.to_string()),
                });
            }
        }
    }

    Ok(OAuthStatus {
        token_path: path,
        token_present: true,
        token_valid: true,
        expires_at: Some(token.expires_at),
        refresh_error: None,
    })
}

/// Resolves client credentials: explicit args → env vars → bundled constants.
pub fn resolve_client_credentials(
    client_id: Option<String>,
    client_secret: Option<String>,
) -> Result<(String, String)> {
    let id = client_id
        .or_else(|| std::env::var("MARINA_GDRIVE_CLIENT_ID").ok())
        .or_else(|| (!BUNDLED_CLIENT_ID.is_empty()).then(|| BUNDLED_CLIENT_ID.to_string()))
        .ok_or_else(|| {
            anyhow!(
                "no OAuth client ID found\n\
            Create a Desktop app at https://console.cloud.google.com/apis/credentials\n\
            then set MARINA_GDRIVE_CLIENT_ID or pass --client-id"
            )
        })?;
    let secret = client_secret
        .or_else(|| std::env::var("MARINA_GDRIVE_CLIENT_SECRET").ok())
        .or_else(|| (!BUNDLED_CLIENT_SECRET.is_empty()).then(|| BUNDLED_CLIENT_SECRET.to_string()))
        .ok_or_else(|| anyhow!("OAuth client secret missing. Set MARINA_GDRIVE_CLIENT_SECRET or pass --client-secret"))?;
    Ok((id, secret))
}

/// Runs the full OAuth2 authorization code flow and stores the resulting token.
pub async fn run_oauth_flow(
    registry_name: &str,
    client_id: &str,
    client_secret: &str,
) -> Result<()> {
    run_oauth_flow_with_options(registry_name, client_id, client_secret, false, None).await
}

/// Runs the full OAuth2 authorization code flow and stores the resulting token.
pub async fn run_oauth_flow_with_options(
    registry_name: &str,
    client_id: &str,
    client_secret: &str,
    no_browser: bool,
    callback_port: Option<u16>,
) -> Result<()> {
    let bind_addr = format!("127.0.0.1:{}", callback_port.unwrap_or(0));
    let listener = tokio::net::TcpListener::bind(&bind_addr)
        .await
        .context("failed to start local callback server")?;
    let port = listener.local_addr()?.port();
    let redirect_uri = format!("http://127.0.0.1:{}/callback", port);

    let consent_url = format!(
        "https://accounts.google.com/o/oauth2/v2/auth\
?client_id={client_id}\
&redirect_uri={encoded_redirect}\
&response_type=code\
&scope={scope}\
&access_type=offline\
&prompt=consent",
        client_id = client_id,
        encoded_redirect = percent_encode(&redirect_uri),
        scope = percent_encode(DRIVE_SCOPE),
    );

    if no_browser {
        eprintln!("Open this URL manually:\n\n{}\n", consent_url);
    } else {
        eprintln!("Opening browser for Google authentication...");
    }
    if !no_browser && !open_browser(&consent_url) {
        eprintln!(
            "Could not open browser automatically. Open this URL manually:\n\n{}\n",
            consent_url
        );
    }

    eprintln!("Waiting for Google to redirect back...");
    let code = wait_for_code(listener).await?;
    eprintln!("Authorization code received, exchanging for tokens...");

    let token = exchange_code(client_id, client_secret, &code, &redirect_uri).await?;
    save_stored_token(registry_name, &token)?;

    eprintln!(
        "Authentication successful. Token saved for registry '{}'.",
        registry_name
    );
    Ok(())
}

async fn wait_for_code(listener: tokio::net::TcpListener) -> Result<String> {
    let (mut stream, _) = listener
        .accept()
        .await
        .context("failed to accept OAuth callback")?;

    let mut buf = [0u8; 4096];
    let n = stream
        .read(&mut buf)
        .await
        .context("failed to read callback request")?;
    let request = std::str::from_utf8(&buf[..n]).unwrap_or("");

    // First line: "GET /callback?code=XXX&... HTTP/1.1"
    let code = request
        .lines()
        .next()
        .and_then(|line| line.split_whitespace().nth(1))
        .and_then(|path| path.split_once('?').map(|(_, qs)| qs))
        .and_then(|qs| {
            qs.split('&')
                .find(|p| p.starts_with("code="))
                .map(|p| p[5..].to_string())
        })
        .ok_or_else(|| anyhow!("no authorization code found in Google callback"))?;

    let html = "<html><body style='font-family:sans-serif;padding:2em'>\
        <h2>Authentication successful!</h2>\
        <p>You can close this tab and return to the terminal.</p>\
        </body></html>";
    let response = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n{}",
        html.len(),
        html
    );
    let _ = stream.write_all(response.as_bytes()).await;

    Ok(code)
}

async fn exchange_code(
    client_id: &str,
    client_secret: &str,
    code: &str,
    redirect_uri: &str,
) -> Result<StoredToken> {
    let client = reqwest::Client::new();
    let resp: serde_json::Value = client
        .post(TOKEN_ENDPOINT)
        .form(&[
            ("code", code),
            ("client_id", client_id),
            ("client_secret", client_secret),
            ("redirect_uri", redirect_uri),
            ("grant_type", "authorization_code"),
        ])
        .send()
        .await
        .context("token exchange request failed")?
        .json()
        .await
        .context("failed to parse token exchange response")?;

    if let Some(err) = resp["error"].as_str() {
        return Err(anyhow!(
            "token exchange failed: {} — {}",
            err,
            resp["error_description"].as_str().unwrap_or("")
        ));
    }

    token_from_response(client_id, client_secret, &resp)
}

async fn do_refresh(
    client_id: &str,
    client_secret: &str,
    refresh_token: &str,
) -> Result<StoredToken> {
    let client = reqwest::Client::new();
    let mut form = vec![
        ("refresh_token", refresh_token),
        ("client_id", client_id),
        ("grant_type", "refresh_token"),
    ];
    if !client_secret.is_empty() {
        form.push(("client_secret", client_secret));
    }
    let resp: serde_json::Value = client
        .post(TOKEN_ENDPOINT)
        .form(&form)
        .send()
        .await
        .context("token refresh request failed")?
        .json()
        .await
        .context("failed to parse token refresh response")?;

    if let Some(err) = resp["error"].as_str() {
        return Err(anyhow!(
            "token refresh failed: {} — {}",
            err,
            resp["error_description"].as_str().unwrap_or("")
        ));
    }

    let access_token = resp["access_token"]
        .as_str()
        .ok_or_else(|| anyhow!("missing access_token in refresh response"))?
        .to_string();

    Ok(build_token(
        client_id,
        client_secret,
        access_token,
        refresh_token.to_string(),
        &resp,
    ))
}

fn token_from_response(
    client_id: &str,
    client_secret: &str,
    resp: &serde_json::Value,
) -> Result<StoredToken> {
    let access_token = resp["access_token"]
        .as_str()
        .ok_or_else(|| anyhow!("missing access_token in response"))?
        .to_string();
    let refresh_token = resp["refresh_token"]
        .as_str()
        .ok_or_else(|| anyhow!("missing refresh_token in response"))?
        .to_string();

    Ok(build_token(
        client_id,
        client_secret,
        access_token,
        refresh_token,
        resp,
    ))
}

fn build_token(
    client_id: &str,
    client_secret: &str,
    access_token: String,
    refresh_token: String,
    resp: &serde_json::Value,
) -> StoredToken {
    let expires_in = resp["expires_in"].as_u64().unwrap_or(3600);
    let expires_at = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
        + expires_in;
    StoredToken {
        client_id: client_id.to_string(),
        client_secret: client_secret.to_string(),
        access_token,
        refresh_token,
        expires_at,
    }
}

fn percent_encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char)
            }
            _ => out.push_str(&format!("%{:02X}", b)),
        }
    }
    out
}

fn open_browser(url: &str) -> bool {
    #[cfg(target_os = "macos")]
    {
        std::process::Command::new("open").arg(url).spawn().is_ok()
    }
    #[cfg(target_os = "linux")]
    {
        std::process::Command::new("xdg-open")
            .arg(url)
            .spawn()
            .is_ok()
    }
    #[cfg(target_os = "windows")]
    {
        std::process::Command::new("cmd")
            .args(["/c", "start", "", url])
            .spawn()
            .is_ok()
    }
    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    {
        false
    }
}

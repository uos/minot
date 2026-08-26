use std::fmt::{Display, Formatter};
use std::str::FromStr;

use anyhow::{Result, anyhow};
use serde::{Deserialize, Serialize};

/// Canonical dataset reference used by marina.
///
/// Examples:
/// - `dlg_cut`
/// - `stelzo/dlg_cut:ouster:1min`
/// - `dlg_cut[traj.txt]`
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub struct BagRef {
    /// Optional namespace (user/project scope).
    pub namespace: Option<String>,
    /// Base bag name.
    pub name: String,
    /// Optional ordered tag chain.
    pub tags: Vec<String>,
    /// Optional attachment path (for exporting/resolving a side file).
    pub attachment: Option<String>,
}

impl BagRef {
    /// Returns a filesystem-safe key used for local cache directories.
    pub fn cache_key(&self) -> String {
        let mut key = String::new();
        if let Some(ns) = &self.namespace {
            key.push_str(ns);
            key.push('_');
        }
        key.push_str(&self.name);
        if !self.tags.is_empty() {
            key.push('_');
            key.push_str(&self.tags.join("_"));
        }
        if let Some(att) = &self.attachment {
            key.push('_');
            key.push_str(att);
        }
        key.replace('/', "_")
    }

    /// Returns the registry object path for this bag (without attachment).
    pub fn object_path(&self) -> String {
        let mut parts = Vec::new();
        if let Some(ns) = &self.namespace {
            parts.push(ns.clone());
        }
        parts.push(self.name.clone());
        for t in &self.tags {
            parts.push(t.clone());
        }
        parts.join("/")
    }

    /// Returns a clone with the given attachment.
    pub fn with_attachment(&self, attachment: Option<String>) -> Self {
        let mut next = self.clone();
        next.attachment = attachment;
        next
    }

    /// Returns a clone with attachment removed.
    pub fn without_attachment(&self) -> Self {
        let mut next = self.clone();
        next.attachment = None;
        next
    }
}

impl Display for BagRef {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        if let Some(ns) = &self.namespace {
            write!(f, "{ns}/")?;
        }
        write!(f, "{}", self.name)?;
        for tag in &self.tags {
            write!(f, ":{tag}")?;
        }
        if let Some(att) = &self.attachment {
            write!(f, "[{att}]")?;
        }
        Ok(())
    }
}

impl FromStr for BagRef {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let (main, attachment) = if let Some(idx) = s.find('[') {
            let close = s
                .rfind(']')
                .ok_or_else(|| anyhow!("attachment syntax is invalid, missing ']'"))?;
            if close < idx {
                return Err(anyhow!("attachment syntax is invalid"));
            }
            let att = s[idx + 1..close].trim();
            (
                s[..idx].trim(),
                if att.is_empty() {
                    None
                } else {
                    Some(att.to_string())
                },
            )
        } else {
            (s.trim(), None)
        };

        if main.is_empty() {
            return Err(anyhow!("bag reference cannot be empty"));
        }

        let slash_idx = main.find('/');
        let (namespace, tail) = match slash_idx {
            Some(i) => {
                let ns = main[..i].trim();
                if ns.is_empty() {
                    return Err(anyhow!("namespace cannot be empty"));
                }
                (Some(ns.to_string()), main[i + 1..].trim())
            }
            None => (None, main),
        };

        let mut parts = tail.split(':');
        let name = parts
            .next()
            .ok_or_else(|| anyhow!("bag name cannot be empty"))?
            .trim();

        if name.is_empty() {
            return Err(anyhow!("bag name cannot be empty"));
        }

        let tags = parts
            .map(str::trim)
            .filter(|v| !v.is_empty())
            .map(ToString::to_string)
            .collect::<Vec<_>>();

        Ok(Self {
            namespace,
            name: name.to_string(),
            tags,
            attachment,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::BagRef;
    use std::str::FromStr;

    #[test]
    fn parse_with_namespace_tags_and_attachment() {
        let bag = BagRef::from_str("stelzo/dlg_cut:ouster:1min[traj.txt]").unwrap();
        assert_eq!(bag.namespace.as_deref(), Some("stelzo"));
        assert_eq!(bag.name, "dlg_cut");
        assert_eq!(bag.tags, vec!["ouster".to_string(), "1min".to_string()]);
        assert_eq!(bag.attachment.as_deref(), Some("traj.txt"));
    }

    #[test]
    fn parse_plain_name() {
        let bag = BagRef::from_str("dlg_cut").unwrap();
        assert_eq!(bag.namespace, None);
        assert_eq!(bag.name, "dlg_cut");
        assert!(bag.tags.is_empty());
        assert!(bag.attachment.is_none());
    }
}

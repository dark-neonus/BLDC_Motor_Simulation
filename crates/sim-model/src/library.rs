//! Parameter library (P04.T08): builtin presets (embedded, read-only), the user library
//! and extra folders from the config file. IDs look like `builtin:motors/8010-outrunner`,
//! `user:motors/my-8010` or `extra1:motors/foo` (1-based extra folder index).

use std::path::{Path, PathBuf};

use rust_embed::RustEmbed;

#[derive(RustEmbed)]
#[folder = "../../presets/"]
#[include = "**/*.yaml"]
struct Presets;

/// Library folder per file kind.
pub const KINDS: [&str; 10] = [
    "motors",
    "gearboxes",
    "loads",
    "inverters",
    "supplies",
    "sensors",
    "controllers",
    "scenes",
    "scenarios",
    "plots",
];

#[derive(Debug, thiserror::Error)]
pub enum LibError {
    #[error("invalid library id `{0}` (expected `source:kind/name`)")]
    BadId(String),
    #[error("invalid name `{0}`: use letters, digits, `-`, `_` or `.`")]
    BadName(String),
    #[error("`{0}` not found")]
    NotFound(String),
    #[error("`{0}` is read-only (builtin and extra entries cannot be changed; duplicate it first)")]
    ReadOnly(String),
    #[error("`{0}` already exists")]
    Exists(String),
    #[error("{0}")]
    Io(#[from] std::io::Error),
    #[error("{0}")]
    Yaml(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    pub id: String,
    pub kind: String,
    pub name: String,
    pub read_only: bool,
}

#[derive(Debug, Clone)]
enum Builtin {
    Embedded,
    Dir(PathBuf),
}

#[derive(Debug, Clone)]
pub struct Library {
    builtin: Builtin,
    user: PathBuf,
    extra: Vec<PathBuf>,
}

#[derive(Debug, Default, serde::Deserialize)]
struct Config {
    #[serde(default)]
    library_extra: Vec<PathBuf>,
}

fn env_dir(var: &str, home_rel: &str) -> PathBuf {
    std::env::var_os(var).map(PathBuf::from).unwrap_or_else(|| {
        PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(home_rel)
    })
}

fn check_name(name: &str) -> Result<(), LibError> {
    let ok = !name.is_empty()
        && !name.starts_with('.')
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || "-_.".contains(c));
    if ok {
        Ok(())
    } else {
        Err(LibError::BadName(name.into()))
    }
}

fn split_id(id: &str) -> Result<(&str, &str, &str), LibError> {
    let bad = || LibError::BadId(id.into());
    let (src, rest) = id.split_once(':').ok_or_else(bad)?;
    let (kind, name) = rest.split_once('/').ok_or_else(bad)?;
    if !KINDS.contains(&kind) {
        return Err(bad());
    }
    check_name(name).map_err(|_| bad())?;
    Ok((src, kind, name))
}

fn yaml_names(dir: &Path) -> Vec<String> {
    let mut v: Vec<String> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            e.file_name()
                .to_str()?
                .strip_suffix(".yaml")
                .map(str::to_owned)
        })
        .filter(|n| !n.starts_with('_'))
        .collect();
    v.sort();
    v
}

impl Library {
    /// Default locations; creates the user library on first run. Extra folders come from
    /// `~/.config/bldc-sim/config.yaml` (`library_extra: [paths]`).
    pub fn open_default() -> Result<Self, LibError> {
        let user = env_dir("XDG_DATA_HOME", ".local/share").join("bldc-sim/library");
        let cfg = env_dir("XDG_CONFIG_HOME", ".config").join("bldc-sim/config.yaml");
        let extra = match std::fs::read_to_string(&cfg) {
            Ok(t) => {
                serde_saphyr::from_str::<Config>(&t)
                    .map_err(|e| LibError::Yaml(format!("{}: {e}", cfg.display())))?
                    .library_extra
            }
            Err(_) => Vec::new(),
        };
        Self::new(None, user, extra)
    }

    /// `builtin_dir: None` uses the embedded presets.
    pub fn new(
        builtin_dir: Option<PathBuf>,
        user: PathBuf,
        extra: Vec<PathBuf>,
    ) -> Result<Self, LibError> {
        for k in KINDS {
            std::fs::create_dir_all(user.join(k))?;
        }
        let builtin = builtin_dir.map_or(Builtin::Embedded, Builtin::Dir);
        Ok(Self {
            builtin,
            user,
            extra,
        })
    }

    fn builtin_names(&self, kind: &str) -> Vec<String> {
        match &self.builtin {
            Builtin::Dir(d) => yaml_names(&d.join(kind)),
            Builtin::Embedded => {
                let mut v: Vec<String> = Presets::iter()
                    .filter_map(|p| {
                        let n = p
                            .strip_prefix(kind)?
                            .strip_prefix('/')?
                            .strip_suffix(".yaml")?;
                        (!n.contains('/') && !n.starts_with('_')).then(|| n.to_owned())
                    })
                    .collect();
                v.sort();
                v
            }
        }
    }

    /// All entries of a kind whose name contains `filter` (case-insensitive).
    pub fn list(&self, kind: &str, filter: &str) -> Vec<Entry> {
        let f = filter.to_lowercase();
        let mut out = Vec::new();
        let mut push = |src: String, names: Vec<String>, read_only: bool| {
            for name in names.into_iter().filter(|n| n.to_lowercase().contains(&f)) {
                out.push(Entry {
                    id: format!("{src}:{kind}/{name}"),
                    kind: kind.into(),
                    name,
                    read_only,
                });
            }
        };
        push("builtin".into(), self.builtin_names(kind), true);
        push("user".into(), yaml_names(&self.user.join(kind)), false);
        for (i, d) in self.extra.iter().enumerate() {
            push(format!("extra{}", i + 1), yaml_names(&d.join(kind)), true);
        }
        out
    }

    fn file(&self, src: &str, kind: &str, name: &str) -> Option<PathBuf> {
        let rel = format!("{kind}/{name}.yaml");
        match src {
            "user" => Some(self.user.join(rel)),
            "builtin" => match &self.builtin {
                Builtin::Dir(d) => Some(d.join(rel)),
                Builtin::Embedded => None,
            },
            s => {
                let i: usize = s.strip_prefix("extra")?.parse().ok()?;
                Some(self.extra.get(i.checked_sub(1)?)?.join(rel))
            }
        }
    }

    /// The raw YAML text of an entry.
    pub fn load(&self, id: &str) -> Result<String, LibError> {
        let (src, kind, name) = split_id(id)?;
        if src == "builtin" && matches!(self.builtin, Builtin::Embedded) {
            let f = Presets::get(&format!("{kind}/{name}.yaml"))
                .ok_or_else(|| LibError::NotFound(id.into()))?;
            return String::from_utf8(f.data.into_owned())
                .map_err(|e| LibError::Yaml(e.to_string()));
        }
        let path = self
            .file(src, kind, name)
            .ok_or_else(|| LibError::BadId(id.into()))?;
        std::fs::read_to_string(path).map_err(|_| LibError::NotFound(id.into()))
    }

    /// Save YAML text into the user library; returns the new id.
    pub fn save_as(
        &self,
        kind: &str,
        name: &str,
        text: &str,
        overwrite: bool,
    ) -> Result<String, LibError> {
        let id = format!("user:{kind}/{name}");
        split_id(&id)?;
        let path = self.user.join(kind).join(format!("{name}.yaml"));
        if path.exists() && !overwrite {
            return Err(LibError::Exists(id));
        }
        std::fs::write(path, text)?;
        Ok(id)
    }

    /// Copy any entry into the user library under a new name; `identity.name` (if the
    /// file has one) is set to the new name.
    pub fn duplicate(&self, id: &str, new_name: &str) -> Result<String, LibError> {
        let (_, kind, _) = split_id(id)?;
        let text = self.load(id)?;
        let mut doc: serde_json::Value =
            serde_saphyr::from_str(&text).map_err(|e| LibError::Yaml(e.to_string()))?;
        let renamed = match doc.pointer_mut("/identity/name") {
            Some(n) => {
                *n = new_name.into();
                true
            }
            None => false,
        };
        let out = if renamed {
            let body = serde_saphyr::to_string(&doc).map_err(|e| LibError::Yaml(e.to_string()))?;
            let modeline: String = text
                .lines()
                .take_while(|l| l.starts_with('#'))
                .map(|l| format!("{l}\n"))
                .collect();
            modeline + &body
        } else {
            text
        };
        self.save_as(kind, new_name, &out, false)
    }

    fn user_path(&self, id: &str) -> Result<(PathBuf, String), LibError> {
        let (src, kind, name) = split_id(id)?;
        if src != "user" {
            return Err(LibError::ReadOnly(id.into()));
        }
        let p = self.user.join(kind).join(format!("{name}.yaml"));
        if !p.exists() {
            return Err(LibError::NotFound(id.into()));
        }
        Ok((p, kind.to_owned()))
    }

    pub fn delete(&self, id: &str) -> Result<(), LibError> {
        let (p, _) = self.user_path(id)?;
        Ok(std::fs::remove_file(p)?)
    }

    /// Rename a user entry; returns the new id.
    pub fn rename(&self, id: &str, new_name: &str) -> Result<String, LibError> {
        let (p, kind) = self.user_path(id)?;
        check_name(new_name)?;
        let new_id = format!("user:{kind}/{new_name}");
        let to = self.user.join(&kind).join(format!("{new_name}.yaml"));
        if to.exists() {
            return Err(LibError::Exists(new_id));
        }
        std::fs::rename(p, to)?;
        Ok(new_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("bldc-lib-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn setup(tag: &str) -> (Library, PathBuf) {
        let root = tmp(tag);
        let b = root.join("builtin/motors");
        std::fs::create_dir_all(&b).unwrap();
        std::fs::write(b.join("8010-outrunner.yaml"), "# yaml-language-server: $schema=x\nidentity: { name: 8010 class, topology: outrunner }\nwinding: { slots: 36, pole_pairs: 21 }\n").unwrap();
        std::fs::write(b.join("_sources.md.yaml"), "x: 1\n").unwrap();
        let x = root.join("extra/motors");
        std::fs::create_dir_all(&x).unwrap();
        std::fs::write(x.join("shop-motor.yaml"), "a: 1\n").unwrap();
        let lib = Library::new(
            Some(root.join("builtin")),
            root.join("user"),
            vec![root.join("extra")],
        )
        .unwrap();
        (lib, root)
    }

    #[test]
    fn list_load_and_builtin_read_only() {
        let (lib, root) = setup("ro");
        let ids: Vec<_> = lib.list("motors", "").into_iter().map(|e| e.id).collect();
        assert_eq!(
            ids,
            ["builtin:motors/8010-outrunner", "extra1:motors/shop-motor"]
        );
        assert_eq!(lib.list("motors", "SHOP").len(), 1);
        assert!(
            lib.load("builtin:motors/8010-outrunner")
                .unwrap()
                .contains("8010 class")
        );
        assert!(matches!(
            lib.delete("builtin:motors/8010-outrunner"),
            Err(LibError::ReadOnly(_))
        ));
        assert!(matches!(
            lib.rename("extra1:motors/shop-motor", "y"),
            Err(LibError::ReadOnly(_))
        ));
        assert!(matches!(
            lib.load("user:motors/../../etc"),
            Err(LibError::BadId(_))
        ));
        assert!(matches!(
            lib.save_as("motors", "../x", "", false),
            Err(LibError::BadId(_))
        ));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn duplicate_rename_delete() {
        let (lib, root) = setup("dup");
        let id = lib
            .duplicate("builtin:motors/8010-outrunner", "my-8010")
            .unwrap();
        assert_eq!(id, "user:motors/my-8010");
        let text = lib.load(&id).unwrap();
        assert!(text.starts_with("# yaml-language-server"), "{text}");
        let orig: serde_json::Value =
            serde_saphyr::from_str(&lib.load("builtin:motors/8010-outrunner").unwrap()).unwrap();
        let mut copy: serde_json::Value = serde_saphyr::from_str(&text).unwrap();
        assert_eq!(copy["identity"]["name"], "my-8010");
        copy["identity"]["name"] = orig["identity"]["name"].clone();
        assert_eq!(copy, orig, "content preserved apart from the name");
        assert!(matches!(
            lib.duplicate("builtin:motors/8010-outrunner", "my-8010"),
            Err(LibError::Exists(_))
        ));
        let id2 = lib.rename(&id, "renamed").unwrap();
        assert_eq!(lib.list("motors", "renamed")[0].id, id2);
        lib.delete(&id2).unwrap();
        assert!(lib.list("motors", "renamed").is_empty());
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn embedded_motor_presets_load_and_validate() {
        let lib = Library::new(None, tmp("emb").join("user"), vec![]).unwrap();
        let motors = lib.list("motors", "");
        assert!(motors.len() >= 9, "{motors:?}");
        for e in motors {
            let text = lib.load(&e.id).unwrap();
            let mut m: crate::params::MotorParams = crate::io::parse_yaml(&text, &e.id).unwrap();
            let mut issues = Vec::new();
            crate::constraints::derive(&mut m, &mut issues);
            issues.extend(crate::constraints::validate(&m));
            let rejects: Vec<_> = issues
                .iter()
                .filter(|i| i.severity == crate::constraints::Severity::Reject)
                .collect();
            assert!(rejects.is_empty(), "{}: {rejects:?}", e.id);
        }
        let _ = std::fs::remove_dir_all(tmp("emb"));
    }
}

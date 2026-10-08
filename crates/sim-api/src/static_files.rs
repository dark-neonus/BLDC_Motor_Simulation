//! Static assets: the web UI at `/` (SPA fallback to index.html) and the docs at
//! `/docs/` — embedded into the binary when built with `embed-ui` / `embed-docs`
//! (`just build`). Without the features, a short hint is served instead.

use axum::Router;
#[cfg(any(feature = "embed-ui", feature = "embed-docs"))]
use axum::http::header;
use axum::http::{StatusCode, Uri};
use axum::response::{IntoResponse, Response};
use axum::routing::get;

use crate::AppState;

pub fn routes() -> Router<AppState> {
    Router::new()
        .route(
            "/docs",
            get(|| async { axum::response::Redirect::permanent("/docs/") }),
        )
        .route("/docs/", get(docs))
        .route("/docs/{*path}", get(docs))
        .fallback(ui)
}

#[cfg(feature = "embed-ui")]
#[derive(rust_embed::Embed)]
#[folder = "../../web/dist"]
struct UiAssets;

#[cfg(feature = "embed-docs")]
#[derive(rust_embed::Embed)]
#[folder = "../../docs/build"]
struct DocsAssets;

#[cfg(any(feature = "embed-ui", feature = "embed-docs"))]
fn asset(file: rust_embed::EmbeddedFile) -> Response {
    (
        [(header::CONTENT_TYPE, file.metadata.mimetype().to_string())],
        file.data,
    )
        .into_response()
}

#[cfg(feature = "embed-ui")]
async fn ui(uri: Uri) -> Response {
    let path = uri.path().trim_start_matches('/');
    if is_api(path) {
        return StatusCode::NOT_FOUND.into_response();
    }
    if let Some(f) = UiAssets::get(path) {
        return asset(f);
    }
    // SPA fallback only for extension-less routes; a missing file (e.g. assets/x.js) is a 404.
    if looks_like_file(path) {
        return StatusCode::NOT_FOUND.into_response();
    }
    match UiAssets::get("index.html") {
        Some(f) => asset(f),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}

#[cfg(not(feature = "embed-ui"))]
async fn ui(uri: Uri) -> Response {
    if is_api(uri.path().trim_start_matches('/')) {
        return StatusCode::NOT_FOUND.into_response();
    }
    (
        StatusCode::NOT_FOUND,
        "Web UI is not embedded in this build. Use `just dev` (UI at http://localhost:5173) or `just build`.",
    )
        .into_response()
}

#[cfg(feature = "embed-docs")]
async fn docs(uri: Uri) -> Response {
    let rel = uri
        .path()
        .trim_start_matches("/docs")
        .trim_start_matches('/');
    let candidates = if rel.is_empty() || rel.ends_with('/') {
        vec![format!("{rel}index.html")]
    } else {
        vec![
            rel.to_string(),
            format!("{rel}/index.html"),
            format!("{rel}.html"),
        ]
    };
    for c in candidates {
        if let Some(f) = DocsAssets::get(&c) {
            return asset(f);
        }
    }
    match DocsAssets::get("404.html") {
        Some(f) => (StatusCode::NOT_FOUND, asset(f)).into_response(),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}

#[cfg(not(feature = "embed-docs"))]
async fn docs() -> Response {
    (
        StatusCode::NOT_FOUND,
        "Docs are not embedded in this build. Use `just docs-dev` (http://localhost:3000/docs/) or `just build`.",
    )
        .into_response()
}

fn is_api(path: &str) -> bool {
    path == "api" || path.starts_with("api/")
}

#[cfg_attr(not(feature = "embed-ui"), allow(dead_code))]
fn looks_like_file(path: &str) -> bool {
    path.rsplit('/')
        .next()
        .is_some_and(|last| last.contains('.'))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn api_and_file_detection() {
        assert!(is_api("api") && is_api("api/x") && !is_api("apis") && !is_api("app/api"));
        assert!(looks_like_file("assets/x.js") && looks_like_file("favicon.svg"));
        assert!(!looks_like_file("some/spa/route") && !looks_like_file(""));
    }
}

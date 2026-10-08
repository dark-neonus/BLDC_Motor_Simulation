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
    if path.starts_with("api/") {
        return StatusCode::NOT_FOUND.into_response();
    }
    match UiAssets::get(path).or_else(|| UiAssets::get("index.html")) {
        Some(f) => asset(f),
        None => StatusCode::NOT_FOUND.into_response(),
    }
}

#[cfg(not(feature = "embed-ui"))]
async fn ui(uri: Uri) -> Response {
    if uri.path().starts_with("/api/") {
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

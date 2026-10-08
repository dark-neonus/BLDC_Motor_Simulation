//! sim-api: the local HTTP/WebSocket server and MCP endpoint that expose the
//! single live simulation (D-006) to the web UI and to AI agents.

pub mod mcp;
pub mod routes;
pub mod static_files;
pub mod ws;

use std::net::SocketAddr;
use std::sync::Arc;

use axum::Router;
use sim_core::skeleton::runner::SimHandle;
use tokio::net::TcpListener;
use tower_http::trace::TraceLayer;

/// Shared server state: the one live simulation.
#[derive(Clone)]
pub struct AppState {
    pub sim: Arc<SimHandle>,
}

/// Build the full router (REST + WebSocket) around a simulation handle.
pub fn router(state: AppState) -> Router {
    let mcp = mcp::service(std::sync::Arc::clone(&state.sim));
    Router::new()
        .nest_service("/mcp", mcp)
        .merge(routes::api())
        .merge(ws::routes())
        .merge(static_files::routes())
        .with_state(state)
        .layer(TraceLayer::new_for_http())
}

/// Serve on an already-bound listener until Ctrl-C (or the future completes).
pub async fn serve(listener: TcpListener, state: AppState) -> std::io::Result<()> {
    axum::serve(listener, router(state))
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await
}

/// Bind `addr`, spawn the simulation and serve (blocking until shutdown).
pub fn run_blocking(addr: SocketAddr, open_browser: bool) -> std::io::Result<()> {
    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(async move {
        let listener = TcpListener::bind(addr).await?;
        tracing::info!("bldc-sim listening on http://{}", listener.local_addr()?);
        let url = format!("http://{}", listener.local_addr()?);
        println!("bldc-sim listening on {url}  (docs: {url}/docs/, MCP: {url}/mcp)");
        if open_browser && let Err(e) = open::that(&url) {
            tracing::warn!("could not open a browser: {e}");
        }
        let state = AppState {
            sim: Arc::new(SimHandle::spawn()),
        };
        serve(listener, state).await
    })
}

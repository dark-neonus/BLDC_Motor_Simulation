//! sim-api: the local HTTP/WebSocket server and MCP endpoint that expose the
//! single live simulation (D-006) to the web UI and to AI agents.

pub mod mcp;
pub mod routes;
pub mod static_files;
pub mod ws;

use std::future::Future;
use std::net::SocketAddr;
use std::sync::Arc;

use axum::Router;
use sim_core::skeleton::runner::SimHandle;
use tokio::net::TcpListener;
use tokio_util::sync::CancellationToken;
use tower_http::trace::TraceLayer;

/// Shared server state: the one live simulation.
#[derive(Clone)]
pub struct AppState {
    pub sim: Arc<SimHandle>,
}

/// Build the full router (MCP + REST + WebSocket + static assets).
pub fn router(state: AppState, shutdown: CancellationToken) -> Router {
    let mcp = mcp::service(Arc::clone(&state.sim), shutdown);
    Router::new()
        .nest_service("/mcp", mcp)
        .merge(routes::api())
        .merge(ws::routes())
        .merge(static_files::routes())
        .with_state(state)
        .layer(TraceLayer::new_for_http())
}

/// Serve on an already-bound listener until `stop` completes. Open MCP streams and
/// WebSockets are closed via a cancellation token so shutdown never hangs.
pub async fn serve_until(
    listener: TcpListener,
    state: AppState,
    stop: impl Future<Output = ()> + Send + 'static,
) -> std::io::Result<()> {
    let token = CancellationToken::new();
    let app = router(state, token.child_token());
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            stop.await;
            token.cancel();
        })
        .await
}

/// Serve until Ctrl-C.
pub async fn serve(listener: TcpListener, state: AppState) -> std::io::Result<()> {
    serve_until(listener, state, async {
        let _ = tokio::signal::ctrl_c().await;
    })
    .await
}

/// Bind `addr`, spawn the simulation and serve (blocking until Ctrl-C).
pub fn run_blocking(addr: SocketAddr, open_browser: bool) -> std::io::Result<()> {
    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(async move {
        let listener = TcpListener::bind(addr).await?;
        let url = format!("http://{}", listener.local_addr()?);
        tracing::info!("bldc-sim listening on {url}");
        println!("bldc-sim listening on {url}  (docs: {url}/docs/, MCP: {url}/mcp)");
        if open_browser && let Err(e) = open::that(&url) {
            tracing::warn!("could not open a browser: {e}");
        }
        let state = AppState {
            sim: Arc::new(SimHandle::spawn()?),
        };
        serve(listener, state).await
    })
}

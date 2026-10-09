//! sim-api: the local HTTP/WebSocket server and MCP endpoint that expose the
//! single live simulation (D-006) to the web UI and to AI agents.

pub mod mcp;
pub mod routes;
pub mod state;
pub mod static_files;
pub mod ws;

use std::future::Future;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use axum::Router;
use sim_core::build::{SceneModel, build_engine as build_scene, reflected_inertia};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::runner::{RunnerCommand, RunnerHandle, RunnerStatus};
use sim_core::skeleton::adapter::{SkeletonOptions, build_engine};
use sim_model::scene::ResolvedScene;
use tokio::net::TcpListener;
use tokio_util::sync::CancellationToken;
use tower_http::trace::TraceLayer;

/// Shared server state: the one live simulation and the scene it was built from.
#[derive(Clone)]
pub struct AppState {
    pub sim: Arc<RunnerHandle>,
    /// The live scene (edits update it, so a reset keeps them). `None`: skeleton defaults.
    pub scene: Option<Arc<Mutex<ResolvedScene>>>,
}

/// Default scene of the live server (P05.T12).
pub const DEFAULT_SCENE: &str = "builtin:scenes/gimbal-hold";

impl AppState {
    /// Run a command on the engine thread and return the status right after it.
    pub async fn command(&self, cmd: RunnerCommand) -> Option<RunnerStatus> {
        let sim = Arc::clone(&self.sim);
        tokio::task::spawn_blocking(move || sim.request(cmd))
            .await
            .ok()
            .flatten()
    }
}

impl AppState {
    /// A live parameter edit. `motor.*` goes through the constraint rules (`apply_edit`)
    /// first; on success the scene is updated and the engine commands are returned.
    pub fn apply_param(
        &self,
        path: &str,
        value: serde_json::Value,
        source: ChangeSource,
    ) -> Result<Vec<EngineCommand>, String> {
        let Some(scene) = &self.scene else {
            return Err("no scene is loaded; parameters cannot be edited".into());
        };
        if !path.starts_with("motor.") {
            return Err(format!(
                "`{path}`: only motor.* parameters are editable live so far"
            ));
        }
        let mut sc = scene
            .lock()
            .map_err(|_| "scene lock poisoned".to_string())?;
        let extra_j =
            reflected_inertia(sc.gearbox.as_ref(), sc.load.as_ref()).map_err(|e| e.to_string())?;
        let mut model = SceneModel {
            motor: sc.motor.clone(),
            extra_j,
        };
        let cmds = model.edit(path, value, source)?;
        sc.motor = model.motor;
        Ok(cmds)
    }
}

/// Resolve the default scene from the library (derived, checked).
fn load_default_scene() -> Result<ResolvedScene, String> {
    use sim_model::io::parse_yaml;
    use sim_model::library::Library;
    let lib = Library::open_default().map_err(|e| e.to_string())?;
    let scene: sim_model::scene::Scene = parse_yaml(
        &lib.load(DEFAULT_SCENE).map_err(|e| e.to_string())?,
        DEFAULT_SCENE,
    )
    .map_err(|e| e.to_string())?;
    let mut r = scene.resolve(&lib).map_err(|e| e.to_string())?;
    r.check(); // derive the motor constants; build_engine re-checks and rejects
    build_scene(&r).map_err(|e| e.to_string())?;
    Ok(r)
}

/// Spawn the default live simulation: the default scene, or the skeleton defaults if it
/// cannot be loaded.
pub fn default_app_state() -> std::io::Result<AppState> {
    match load_default_scene() {
        Ok(scene) => {
            let shared = Arc::new(Mutex::new(scene));
            let s2 = Arc::clone(&shared);
            // INVARIANT: the scene built once above, and edits pass the constraint rules
            // before they reach it, so rebuilding on reset cannot fail; fall back anyway.
            let factory = Box::new(move || {
                let sc = s2.lock().map(|g| g.clone());
                match sc.ok().map(|s| build_scene(&s)) {
                    Some(Ok(b)) => b.engine,
                    _ => build_engine(SkeletonOptions::default()),
                }
            });
            Ok(AppState {
                sim: Arc::new(RunnerHandle::spawn(factory, None)?),
                scene: Some(shared),
            })
        }
        Err(e) => {
            tracing::warn!("default scene unavailable ({e}); using the skeleton defaults");
            Ok(AppState {
                sim: Arc::new(default_runner()?),
                scene: None,
            })
        }
    }
}

/// Spawn the skeleton live simulation (fallback and tests).
pub fn default_runner() -> std::io::Result<RunnerHandle> {
    RunnerHandle::spawn(Box::new(|| build_engine(SkeletonOptions::default())), None)
}

/// Build the full router (MCP + REST + WebSocket + static assets).
pub fn router(state: AppState, shutdown: CancellationToken) -> Router {
    let mcp = mcp::service(state.clone(), shutdown);
    Router::new()
        .nest_service("/mcp", mcp)
        .merge(routes::api())
        .merge(ws::routes())
        .merge(static_files::routes())
        .with_state(state)
        .layer(TraceLayer::new_for_http())
}

/// Serve until `stop` completes; open MCP streams are cancelled so shutdown never hangs.
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
        let state = default_app_state()?;
        serve(listener, state).await
    })
}

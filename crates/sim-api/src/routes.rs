//! REST routes (skeleton subset of the P12 API; paths are kept stable).

use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::Deserialize;
use sim_core::skeleton::runner::{Command, StateSnapshot};

use crate::AppState;

/// Body for endpoints that take a single number.
#[derive(Debug, Deserialize)]
pub struct Value {
    pub value: f64,
}

pub fn api() -> Router<AppState> {
    Router::new()
        .route("/api/health", get(health))
        .route("/api/state", get(state))
        .route("/api/sim/play", post(play))
        .route("/api/sim/pause", post(pause))
        .route("/api/sim/reset", post(reset))
        .route("/api/sim/time_scale", post(time_scale))
        .route("/api/ctrl/target_speed", post(target_speed))
}

async fn health() -> Json<serde_json::Value> {
    Json(serde_json::json!({ "status": "ok", "version": env!("CARGO_PKG_VERSION") }))
}

async fn state(State(s): State<AppState>) -> Json<StateSnapshot> {
    Json(s.sim.state())
}

/// Send a command and return the state right after the runner applied it.
async fn command(
    s: &AppState,
    cmd: Command,
) -> Result<Json<StateSnapshot>, (StatusCode, &'static str)> {
    let sim = std::sync::Arc::clone(&s.sim);
    tokio::task::spawn_blocking(move || sim.request(cmd))
        .await
        .ok()
        .flatten()
        .map(Json)
        .ok_or((
            StatusCode::SERVICE_UNAVAILABLE,
            "simulation runner did not respond",
        ))
}

type Reply = Result<Json<StateSnapshot>, (StatusCode, &'static str)>;

async fn play(State(s): State<AppState>) -> Reply {
    command(&s, Command::Play).await
}
async fn pause(State(s): State<AppState>) -> Reply {
    command(&s, Command::Pause).await
}
async fn reset(State(s): State<AppState>) -> Reply {
    command(&s, Command::Reset).await
}
async fn time_scale(State(s): State<AppState>, Json(v): Json<Value>) -> Reply {
    command(&s, Command::SetTimeScale(v.value)).await
}
async fn target_speed(State(s): State<AppState>, Json(v): Json<Value>) -> Reply {
    command(&s, Command::SetTargetSpeed(v.value)).await
}

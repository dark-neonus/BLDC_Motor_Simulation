//! REST routes (skeleton subset of the P12 API; paths are kept stable).

use axum::extract::State;
use axum::http::StatusCode;
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::{Map, Value as Json_};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::runner::RunnerCommand;

use crate::AppState;
use crate::state::state_map;

/// Body for endpoints that take a single number.
#[derive(Debug, Deserialize)]
pub struct Value {
    pub value: f64,
}

type Reply = Result<Json<Map<String, Json_>>, (StatusCode, &'static str)>;

pub fn api() -> Router<AppState> {
    Router::new()
        .route("/api/health", get(health))
        .route("/api/state", get(state))
        .route("/api/sim/play", post(play))
        .route("/api/sim/pause", post(pause))
        .route("/api/sim/reset", post(reset))
        .route("/api/sim/time_scale", post(time_scale))
        .route("/api/ctrl/target_speed", post(target_speed))
        .route("/api/param", post(set_param))
}

/// Body of `POST /api/param`: a dotted parameter path and a value (number or quantity
/// string such as "0.2 ohm").
#[derive(Debug, Deserialize)]
pub struct ParamEdit {
    pub path: String,
    pub value: Json_,
}

/// Live parameter edit through the constraint rules; 422 with the reason if rejected.
async fn set_param(
    State(s): State<AppState>,
    Json(p): Json<ParamEdit>,
) -> Result<Json<Map<String, Json_>>, (StatusCode, Json<Json_>)> {
    let cmds = s
        .apply_param(&p.path, p.value, ChangeSource::Ui)
        .map_err(|e| {
            (
                StatusCode::UNPROCESSABLE_ENTITY,
                Json(serde_json::json!({ "error": e })),
            )
        })?;
    let mut last = None;
    for c in cmds {
        last = s.command(RunnerCommand::Engine(c)).await;
    }
    let st = match last {
        Some(st) => st,
        None => s.sim.status(),
    };
    Ok(Json(state_map(&s.sim, &st)))
}

async fn health() -> Json<Json_> {
    Json(serde_json::json!({ "status": "ok", "version": env!("CARGO_PKG_VERSION") }))
}

async fn state(State(s): State<AppState>) -> Json<Map<String, Json_>> {
    Json(state_map(&s.sim, &s.sim.status()))
}

async fn command(s: &AppState, cmd: RunnerCommand) -> Reply {
    s.command(cmd)
        .await
        .map(|st| Json(state_map(&s.sim, &st)))
        .ok_or((
            StatusCode::SERVICE_UNAVAILABLE,
            "simulation runner did not respond",
        ))
}

async fn play(State(s): State<AppState>) -> Reply {
    command(&s, RunnerCommand::Play).await
}
async fn pause(State(s): State<AppState>) -> Reply {
    command(&s, RunnerCommand::Pause).await
}
async fn reset(State(s): State<AppState>) -> Reply {
    command(&s, RunnerCommand::Reset).await
}
async fn time_scale(State(s): State<AppState>, Json(v): Json<Value>) -> Reply {
    command(&s, RunnerCommand::TimeScale(v.value)).await
}
async fn target_speed(State(s): State<AppState>, Json(v): Json<Value>) -> Reply {
    let cmd = EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: v.value,
        source: ChangeSource::Ui,
    };
    command(&s, RunnerCommand::Engine(cmd)).await
}

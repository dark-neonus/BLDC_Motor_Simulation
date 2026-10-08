//! WebSocket stream `/api/stream`: MessagePack frames at 30 Hz.
//! Frame: a map `{type: "state", v: 1, ...StateSnapshot fields}`.

use std::time::Duration;

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::response::Response;
use axum::routing::get;
use serde::Serialize;
use sim_core::skeleton::runner::StateSnapshot;

use crate::AppState;

const FRAME_PERIOD: Duration = Duration::from_millis(33);

#[derive(Serialize)]
struct StateFrame {
    #[serde(rename = "type")]
    kind: &'static str,
    /// Protocol version.
    v: u32,
    #[serde(flatten)]
    state: StateSnapshot,
}

pub fn routes() -> Router<AppState> {
    Router::new().route("/api/stream", get(upgrade))
}

async fn upgrade(ws: WebSocketUpgrade, State(s): State<AppState>) -> Response {
    ws.on_upgrade(move |socket| stream(socket, s))
}

async fn stream(mut socket: WebSocket, s: AppState) {
    let mut tick = tokio::time::interval(FRAME_PERIOD);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tick.tick().await;
        let frame = StateFrame {
            kind: "state",
            v: 1,
            state: s.sim.state(),
        };
        let Ok(bytes) = rmp_serde::to_vec_named(&frame) else {
            tracing::error!("failed to encode state frame");
            return;
        };
        if socket.send(Message::Binary(bytes.into())).await.is_err() {
            return; // client went away
        }
    }
}

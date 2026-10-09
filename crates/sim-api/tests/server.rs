//! Integration test: REST control + WebSocket MessagePack stream.

use std::sync::Arc;
use std::time::Duration;

use futures_util::StreamExt;
use sim_api::default_runner;
use sim_api::{AppState, serve};
use tokio::net::TcpListener;

async fn start() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let state = AppState {
        sim: Arc::new(default_runner().unwrap()),
        scene: None,
    };
    tokio::spawn(serve(listener, state));
    format!("127.0.0.1:{}", addr.port())
}

#[tokio::test]
async fn rest_controls_sim_and_ws_streams_frames() {
    let addr = start().await;
    let http = reqwest::Client::new();

    let health: serde_json::Value = http
        .get(format!("http://{addr}/api/health"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(health["status"], "ok");

    http.post(format!("http://{addr}/api/ctrl/target_speed"))
        .json(&serde_json::json!({ "value": 10.0 }))
        .send()
        .await
        .unwrap();
    let st: serde_json::Value = http
        .post(format!("http://{addr}/api/sim/play"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(st["sim.running"], true);
    assert_eq!(st["ctrl.omega_ref"], 10.0);

    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/stream"))
        .await
        .unwrap();
    let mut frames = 0;
    let mut last_t = -1.0;
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    while frames < 5 && tokio::time::Instant::now() < deadline {
        let Some(Ok(msg)) = ws.next().await else {
            break;
        };
        if let tokio_tungstenite::tungstenite::Message::Binary(b) = msg {
            let v: serde_json::Value = rmp_serde::from_slice(&b).unwrap();
            assert_eq!(v["type"], "state");
            let t = v["sim.t"].as_f64().unwrap();
            assert!(t >= last_t, "time must not go backwards");
            last_t = t;
            frames += 1;
        }
    }
    assert!(frames >= 5, "received only {frames} frames");
    assert!(last_t > 0.0, "sim time should advance while playing");
}

#[tokio::test]
async fn live_param_edits_pass_the_constraint_rules() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
    let state = sim_api::default_app_state().unwrap();
    let scene = state.scene.clone().expect("the default scene loads");
    let r0 = scene.lock().unwrap().motor.electrical.r_phase.clone();
    tokio::spawn(serve(listener, state));
    let http = reqwest::Client::new();
    let post = |v: serde_json::Value| {
        http.post(format!("http://{addr}/api/param"))
            .json(&serde_json::json!({ "path": "motor.electrical.r_phase", "value": v }))
            .send()
    };

    // Rejected: negative resistance. The reason comes back; nothing changes.
    let bad = post(serde_json::json!("-1 ohm")).await.unwrap();
    assert_eq!(bad.status(), 422);
    let body: serde_json::Value = bad.json().await.unwrap();
    assert!(
        body["error"].as_str().unwrap().contains("r_phase"),
        "{body}"
    );
    assert_eq!(scene.lock().unwrap().motor.electrical.r_phase, r0);

    // Accepted: the scene (and so a later reset) carries the new value.
    let ok = post(serde_json::json!("6 ohm")).await.unwrap();
    assert_eq!(ok.status(), 200);
    assert_eq!(
        scene.lock().unwrap().motor.electrical.r_phase.value,
        sim_model::param::Raw::Text("6 ohm".into())
    );

    // Non-motor paths are refused for now.
    let other = http
        .post(format!("http://{addr}/api/param"))
        .json(&serde_json::json!({ "path": "inverter.f_pwm", "value": 1000.0 }))
        .send()
        .await
        .unwrap();
    assert_eq!(other.status(), 422);
}

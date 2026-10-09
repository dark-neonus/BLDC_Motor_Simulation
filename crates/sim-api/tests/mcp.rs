//! Integration tests: MCP over streamable HTTP drives the same live simulation
//! that REST exposes, and an open MCP session does not block shutdown.

use std::sync::Arc;
use std::time::Duration;

use rmcp::ServiceExt;
use rmcp::model::{CallToolRequestParams, ClientConfig};
use rmcp::transport::StreamableHttpClientTransport;
use rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig;
use sim_api::default_runner;
use sim_api::{AppState, serve_until};
use tokio::net::TcpListener;
use tokio::sync::oneshot;

async fn start() -> (
    String,
    oneshot::Sender<()>,
    tokio::task::JoinHandle<std::io::Result<()>>,
) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let state = AppState {
        sim: Arc::new(default_runner().unwrap()),
        scene: None,
    };
    let (stop_tx, stop_rx) = oneshot::channel::<()>();
    let server = tokio::spawn(serve_until(listener, state, async {
        let _ = stop_rx.await;
    }));
    (format!("127.0.0.1:{}", addr.port()), stop_tx, server)
}

async fn connect(addr: &str) -> rmcp::service::RunningService<rmcp::RoleClient, ClientConfig> {
    let transport = StreamableHttpClientTransport::from_config(
        StreamableHttpClientTransportConfig::with_uri(format!("http://{addr}/mcp")),
    );
    ClientConfig::default().serve(transport).await.unwrap()
}

fn args(v: serde_json::Value) -> serde_json::Map<String, serde_json::Value> {
    serde_json::from_value(v).unwrap()
}

#[tokio::test]
async fn mcp_drives_the_same_sim_that_rest_reports() -> anyhow::Result<()> {
    let (addr, stop, server) = start().await;
    let client = connect(&addr).await;

    let names: Vec<String> = client
        .list_all_tools()
        .await?
        .iter()
        .map(|t| t.name.to_string())
        .collect();
    for expected in ["get_state", "sim_control", "set_target_speed"] {
        assert!(
            names.iter().any(|n| n == expected),
            "missing tool {expected}: {names:?}"
        );
    }

    let target = 12.5;
    let r = client
        .call_tool(
            CallToolRequestParams::new("set_target_speed")
                .with_arguments(args(serde_json::json!({ "rad_per_s": target }))),
        )
        .await?;
    assert_eq!(
        r.structured_content.as_ref().unwrap()["ctrl.omega_ref"],
        target
    );
    client
        .call_tool(
            CallToolRequestParams::new("sim_control")
                .with_arguments(args(serde_json::json!({ "action": "play" }))),
        )
        .await?;

    // The motor follows the setpoint: poll get_state until ω > half the target.
    let mut omega = 0.0;
    for _ in 0..100 {
        let r = client
            .call_tool(CallToolRequestParams::new("get_state"))
            .await?;
        omega = r.structured_content.as_ref().unwrap()["motor.omega"]
            .as_f64()
            .unwrap();
        if omega > 0.5 * target {
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert!(
        omega > 0.5 * target,
        "motor did not follow the MCP setpoint, omega = {omega}"
    );

    // REST sees the same instance (D-006).
    let rest: serde_json::Value = reqwest::get(format!("http://{addr}/api/state"))
        .await?
        .json()
        .await?;
    assert_eq!(rest["ctrl.omega_ref"], target);
    assert_eq!(rest["sim.running"], true);

    // Invalid action is rejected (schema enum), not reported as success.
    let bad = client
        .call_tool(
            CallToolRequestParams::new("sim_control")
                .with_arguments(args(serde_json::json!({ "action": "explode" }))),
        )
        .await;
    assert!(
        bad.is_err() || bad.as_ref().is_ok_and(|r| r.is_error == Some(true)),
        "bad action accepted: {bad:?}"
    );

    let _ = client.cancel().await;
    let _ = stop.send(());
    tokio::time::timeout(Duration::from_secs(5), server)
        .await??
        .unwrap();
    Ok(())
}

#[tokio::test]
async fn open_mcp_session_does_not_block_shutdown() -> anyhow::Result<()> {
    let (addr, stop, server) = start().await;
    let client = connect(&addr).await; // keeps a session (and possibly an SSE stream) open
    client
        .call_tool(CallToolRequestParams::new("get_state"))
        .await?;
    let _ = stop.send(());
    tokio::time::timeout(Duration::from_secs(5), server)
        .await
        .expect("server must shut down within 5 s with an open MCP session")??;
    drop(client);
    Ok(())
}

//! Integration test: MCP over streamable HTTP drives the same live simulation.

use std::sync::Arc;

use rmcp::ServiceExt;
use rmcp::model::{CallToolRequestParams, ClientConfig};
use rmcp::transport::StreamableHttpClientTransport;
use rmcp::transport::streamable_http_client::StreamableHttpClientTransportConfig;
use sim_api::{AppState, serve};
use sim_core::skeleton::runner::SimHandle;
use tokio::net::TcpListener;

fn text_of(result: &rmcp::model::CallToolResult) -> String {
    result
        .content
        .iter()
        .filter_map(|c| c.as_text().map(|t| t.text.clone()))
        .collect::<Vec<_>>()
        .join("")
}

#[tokio::test]
async fn mcp_tools_list_and_set_target_speed() -> anyhow::Result<()> {
    let listener = TcpListener::bind("127.0.0.1:0").await?;
    let addr = listener.local_addr()?;
    let sim = Arc::new(SimHandle::spawn());
    tokio::spawn(serve(
        listener,
        AppState {
            sim: Arc::clone(&sim),
        },
    ));

    let transport = StreamableHttpClientTransport::from_config(
        StreamableHttpClientTransportConfig::with_uri(format!("http://{addr}/mcp")),
    );
    let client = ClientConfig::default().serve(transport).await?;

    let tools = client.list_all_tools().await?;
    let names: Vec<String> = tools.iter().map(|t| t.name.to_string()).collect();
    for expected in ["get_state", "sim_control", "set_target_speed"] {
        assert!(
            names.iter().any(|n| n == expected),
            "missing tool {expected}: {names:?}"
        );
    }

    let args = serde_json::from_value(serde_json::json!({ "rad_per_s": 12.5 }))?;
    let r = client
        .call_tool(CallToolRequestParams::new("set_target_speed").with_arguments(args))
        .await?;
    assert!(r.is_error != Some(true));

    let r = client
        .call_tool(CallToolRequestParams::new("get_state"))
        .await?;
    let state: serde_json::Value = serde_json::from_str(&text_of(&r))?;
    assert_eq!(state["omega_ref"], 12.5);
    // Same instance as REST (D-006).
    assert_eq!(sim.state().omega_ref, 12.5);

    let _ = client.cancel().await;
    Ok(())
}

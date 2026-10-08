//! MCP server (streamable HTTP at `/mcp`) — skeleton tool set (P01.T06).
//! Tools operate on the same live simulation as the REST API (D-006).

use std::sync::Arc;
use std::time::Duration;

use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{ServerCapabilities, ServerConfig};
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::{StreamableHttpServerConfig, StreamableHttpService};
use rmcp::{ServerHandler, schemars, tool, tool_handler, tool_router};
use serde::Deserialize;
use sim_core::skeleton::runner::{Command, SimHandle};

/// Arguments of `sim_control`.
#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct SimControlArgs {
    /// One of "play", "pause", "reset".
    pub action: String,
}

/// Arguments of `set_target_speed`.
#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct TargetSpeedArgs {
    /// Mechanical speed setpoint in rad/s (100 rpm = 10.47 rad/s). The skeleton motor
    /// tops out near 33 rad/s at its 24 V bus.
    pub rad_per_s: f64,
}

/// MCP tool server bound to the live simulation.
#[derive(Clone)]
pub struct BldcMcp {
    sim: Arc<SimHandle>,
    tool_router: ToolRouter<Self>,
}

#[tool_router]
impl BldcMcp {
    pub fn new(sim: Arc<SimHandle>) -> Self {
        Self {
            sim,
            tool_router: Self::tool_router(),
        }
    }

    async fn settled_state_json(&self) -> String {
        // The runner publishes right after applying a command.
        tokio::time::sleep(Duration::from_millis(5)).await;
        serde_json::to_string(&self.sim.state())
            .unwrap_or_else(|e| format!("{{\"error\":\"{e}\"}}"))
    }

    #[tool(
        description = "Get the live simulation state as JSON: t [s], omega [rad/s], theta [rad], i_d/i_q/i_a/i_b/i_c [A], omega_ref [rad/s], running, time_scale, sim_real_ratio."
    )]
    async fn get_state(&self) -> String {
        self.settled_state_json().await
    }

    #[tool(
        description = "Control the simulation clock: action = \"play\", \"pause\" or \"reset\". Returns the new state."
    )]
    async fn sim_control(
        &self,
        Parameters(SimControlArgs { action }): Parameters<SimControlArgs>,
    ) -> String {
        let cmd = match action.as_str() {
            "play" => Command::Play,
            "pause" => Command::Pause,
            "reset" => Command::Reset,
            other => {
                return format!(
                    "{{\"error\":\"unknown action '{other}', expected play|pause|reset\"}}"
                );
            }
        };
        self.sim.send(cmd);
        self.settled_state_json().await
    }

    #[tool(
        description = "Set the motor speed setpoint in rad/s (velocity control). Returns the new state."
    )]
    async fn set_target_speed(
        &self,
        Parameters(TargetSpeedArgs { rad_per_s }): Parameters<TargetSpeedArgs>,
    ) -> String {
        self.sim.send(Command::SetTargetSpeed(rad_per_s));
        self.settled_state_json().await
    }
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for BldcMcp {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(ServerCapabilities::builder().enable_tools().build())
    }
}

/// The tower service to mount at `/mcp`.
pub fn service(sim: Arc<SimHandle>) -> StreamableHttpService<BldcMcp, LocalSessionManager> {
    StreamableHttpService::new(
        move || Ok(BldcMcp::new(Arc::clone(&sim))),
        Default::default(),
        StreamableHttpServerConfig::default(),
    )
}

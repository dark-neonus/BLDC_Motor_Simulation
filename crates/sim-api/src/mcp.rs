//! MCP server (streamable HTTP at `/mcp`) — skeleton tool set (P01.T06).
//! Tools operate on the same live simulation as the REST API (D-006) and return
//! the state both as text and as structured content (dotted signal names).

use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ServerCapabilities, ServerConfig};
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::{StreamableHttpServerConfig, StreamableHttpService};
use rmcp::{ErrorData, ServerHandler, schemars, tool, tool_handler, tool_router};
use serde::Deserialize;
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::runner::RunnerCommand;

use crate::AppState;
use crate::state::state_map;
use tokio_util::sync::CancellationToken;

/// Clock action for `sim_control`.
#[derive(Debug, Clone, Copy, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ClockAction {
    Play,
    Pause,
    Reset,
}

/// Arguments of `sim_control`.
#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct SimControlArgs {
    /// What to do with the simulation clock.
    pub action: ClockAction,
}

/// Arguments of `set_target_speed`.
#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct TargetSpeedArgs {
    /// Mechanical speed setpoint in rad/s (100 rpm = 10.47 rad/s). The skeleton motor
    /// tops out near 33 rad/s at its 24 V bus.
    pub rad_per_s: f64,
}

/// Arguments of `set_param`.
#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
pub struct SetParamArgs {
    /// Dotted parameter path, e.g. `motor.electrical.r_phase`.
    pub path: String,
    /// A number (SI) or a quantity string such as "0.2 ohm".
    pub value: serde_json::Value,
}

/// MCP tool server bound to the live simulation.
#[derive(Clone)]
pub struct BldcMcp {
    app: AppState,
    tool_router: ToolRouter<Self>,
}

#[tool_router]
impl BldcMcp {
    pub fn new(app: AppState) -> Self {
        Self {
            app,
            tool_router: Self::tool_router(),
        }
    }

    /// Run a command on the runner and return the state right after it was applied.
    async fn command(&self, cmd: RunnerCommand) -> Result<CallToolResult, ErrorData> {
        let st =
            self.app.command(cmd).await.ok_or_else(|| {
                ErrorData::internal_error("simulation runner did not respond", None)
            })?;
        structured(&state_map(&self.app.sim, &st))
    }

    #[tool(
        description = "Get the live simulation state. Keys are signal paths: sim.t [s], motor.omega [rad/s], motor.theta [rad], motor.i_d/i_q/i_a/i_b/i_c [A], ctrl.omega_ref [rad/s], sim.running, sim.time_scale, sim.real_ratio."
    )]
    async fn get_state(&self) -> Result<CallToolResult, ErrorData> {
        structured(&state_map(&self.app.sim, &self.app.sim.status()))
    }

    #[tool(
        description = "Control the simulation clock (play, pause or reset). Returns the state after the action."
    )]
    async fn sim_control(
        &self,
        Parameters(SimControlArgs { action }): Parameters<SimControlArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let cmd = match action {
            ClockAction::Play => RunnerCommand::Play,
            ClockAction::Pause => RunnerCommand::Pause,
            ClockAction::Reset => RunnerCommand::Reset,
        };
        self.command(cmd).await
    }

    #[tool(
        description = "Edit a live parameter by its dotted path (e.g. motor.electrical.r_phase) with a number (SI) or a quantity string (\"0.2 ohm\"). Edits pass the constraint rules first; a rejected edit returns the reason and changes nothing. Only motor.* is editable live so far."
    )]
    async fn set_param(
        &self,
        Parameters(SetParamArgs { path, value }): Parameters<SetParamArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        let cmds = self
            .app
            .apply_param(&path, value, ChangeSource::Mcp)
            .map_err(|e| ErrorData::invalid_params(e, None))?;
        for c in cmds {
            self.app.command(RunnerCommand::Engine(c)).await;
        }
        structured(&state_map(&self.app.sim, &self.app.sim.status()))
    }

    #[tool(
        description = "Set the motor speed setpoint in rad/s (velocity control). Returns the state after the change."
    )]
    async fn set_target_speed(
        &self,
        Parameters(TargetSpeedArgs { rad_per_s }): Parameters<TargetSpeedArgs>,
    ) -> Result<CallToolResult, ErrorData> {
        if !rad_per_s.is_finite() {
            return Err(ErrorData::invalid_params(
                "rad_per_s must be a finite number",
                None,
            ));
        }
        let cmd = EngineCommand::SetSignal {
            path: "ctrl.omega_ref".into(),
            value: rad_per_s,
            source: ChangeSource::Mcp,
        };
        self.command(RunnerCommand::Engine(cmd)).await
    }
}

fn structured<T: serde::Serialize>(value: &T) -> Result<CallToolResult, ErrorData> {
    let v =
        serde_json::to_value(value).map_err(|e| ErrorData::internal_error(e.to_string(), None))?;
    Ok(CallToolResult::structured(v))
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for BldcMcp {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(ServerCapabilities::builder().enable_tools().build())
    }
}

/// The tower service to mount at `/mcp`. Cancelling `shutdown` closes open MCP
/// streams so graceful server shutdown can complete.
pub fn service(
    app: AppState,
    shutdown: CancellationToken,
) -> StreamableHttpService<BldcMcp, LocalSessionManager> {
    StreamableHttpService::new(
        move || Ok(BldcMcp::new(app.clone())),
        Default::default(),
        StreamableHttpServerConfig::default().with_cancellation_token(shutdown),
    )
}

//! `bldc-sim` — command-line entry point.

mod cli;

use std::path::PathBuf;

use clap::{Parser, Subcommand};

/// BLDC motor simulator.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Option<Cmd>,
}

#[derive(Subcommand)]
enum Cmd {
    /// Start the local server (REST + WebSocket + MCP) with the live simulation.
    Serve {
        /// TCP port to listen on.
        #[arg(long, default_value_t = 8787)]
        port: u16,
        /// Address to bind (localhost only by default).
        #[arg(long, default_value = "127.0.0.1")]
        host: std::net::IpAddr,
    },
    /// Run a scenario file headless and write signals.parquet, signals.csv and meta.json.
    RunScenario {
        /// Scenario YAML file.
        file: PathBuf,
        /// Output directory.
        #[arg(long, default_value = "out")]
        out: PathBuf,
    },
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,tower_http=warn".into()),
        )
        .init();
    let cli = Cli::parse();
    match cli.command {
        Some(Cmd::Serve { port, host }) => Ok(sim_api::run_blocking((host, port).into())?),
        Some(Cmd::RunScenario { file, out }) => cli::run_scenario::run(&file, &out),
        None => {
            println!(
                "bldc-sim {} — see `bldc-sim --help`",
                env!("CARGO_PKG_VERSION")
            );
            Ok(())
        }
    }
}

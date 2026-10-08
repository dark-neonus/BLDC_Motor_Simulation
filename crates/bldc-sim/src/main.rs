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
    let cli = Cli::parse();
    match cli.command {
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

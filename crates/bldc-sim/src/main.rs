//! `bldc-sim` — command-line entry point (serve, run-scenario, ...).

use clap::Parser;

/// BLDC motor simulator.
#[derive(Parser)]
#[command(version, about)]
struct Cli {}

fn main() {
    let _cli = Cli::parse();
    println!("bldc-sim {}", env!("CARGO_PKG_VERSION"));
}

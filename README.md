# BLDC Motor Simulator

A local, browser-based simulator and learning lab for brushless permanent-magnet motors (BLDC/PMSM), focused on high-torque, low-RPM motors (gimbal motors, robot actuators). It has a validated physics core in Rust, a live web UI with a motor visualization and plots, configurable control loops, sensors, power supplies and loads, full docs, and an MCP server so AI agents can drive the running app.

**Status:** under construction. See [Context/PLAN.md](Context/PLAN.md) for the plan and progress dashboard.

## Development

- Toolchain: [mise](https://mise.jdx.dev) — `mise install` installs the pinned versions from `mise.toml`.
- Tasks: [just](https://just.systems) — `just --list` (available after phase 00).
- Agents and contributors: start with [Context/PLAN.md](Context/PLAN.md).

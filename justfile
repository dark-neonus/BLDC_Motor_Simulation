# BLDC simulator task runner. `just --list` shows all recipes.
# Recipes run in bash with the mise shims on PATH, so they work from fish, bash or CI.

set shell := ["bash", "-euo", "pipefail", "-c"]
set export

PATH := env("HOME") + "/.local/share/mise/shims:" + env("HOME") + "/.local/bin:" + env("PATH")
BLDC_SIM_BIN := justfile_directory() + "/target/release/bldc-sim"

# List recipes
default:
    @just --list

# ---------------------------------------------------------------- setup

# Install toolchains and all dependencies
setup:
    mise trust --yes
    mise install
    cargo fetch
    pnpm install
    cd validation && uv sync
    @if [ -f web/playwright.config.ts ]; then pnpm -C web exec playwright install chromium; else echo "SKIP playwright browsers (until P01.T11)"; fi

# ---------------------------------------------------------------- run

# Run server (:8787) and web dev server (:5173) together
dev:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! cargo run -q -p bldc-sim -- serve --help >/dev/null 2>&1; then
        echo "SKIP dev server (until P01.T05); starting web only"; exec pnpm -C web dev
    fi
    trap 'kill 0' EXIT INT TERM
    cargo run -p bldc-sim -- serve &
    pnpm -C web dev &
    wait

# Docs dev server (:3000)
docs-dev:
    @if [ -f docs/package.json ]; then pnpm -C docs start; else echo "SKIP docs (until P01.T08)"; fi

# Release build: single binary (embeds UI/docs once P01.T10 lands)
build:
    pnpm -C web build
    @if [ -f docs/package.json ]; then pnpm -C docs build; else echo "SKIP docs build (until P01.T08)"; fi
    cargo build --release -p bldc-sim

# ---------------------------------------------------------------- tests

# All tests (Rust, web, Python validation)
test: test-rust test-web validate

# Rust tests via nextest
test-rust:
    cargo nextest run --workspace --no-tests=pass

# Web unit tests (vitest)
test-web:
    pnpm -C web test --run

# Python validation suite (builds release binary first); extra args go to pytest
validate *ARGS:
    cargo build --release -p bldc-sim
    cd validation && uv run pytest -q {{ARGS}}

# Playwright end-to-end tests; extra args go to playwright
e2e *ARGS:
    @if [ -f web/playwright.config.ts ]; then pnpm -C web exec playwright test {{ARGS}}; else echo "SKIP e2e (until P01.T11)"; fi

# Criterion benchmarks
bench:
    @if ls crates/*/benches/*.rs >/dev/null 2>&1; then cargo bench --workspace; else echo "SKIP bench (until P03.T13)"; fi

# ---------------------------------------------------------------- quality

# Format everything
fmt:
    cargo fmt --all
    pnpm biome format --write .
    cd validation && uv run ruff format .

# Lint everything (no changes)
lint:
    cargo clippy --workspace --all-targets -- -D warnings
    pnpm biome check .
    pnpm -C web exec tsc -b --noEmit
    cd validation && uv run ruff check .

# Format checks only (used by check-fast)
fmt-check:
    cargo fmt --all --check
    cd validation && uv run ruff format --check .

# Fast gate, run after every task
check-fast: fmt-check lint test-rust test-web plan-check

# Full gate, run at phase gates
check: check-fast validate e2e
    pnpm -C web build
    @if [ -f docs/package.json ]; then pnpm -C docs build; else echo "SKIP docs build (until P01.T08)"; fi

# ---------------------------------------------------------------- codegen

# Regenerate TS API types from the server's OpenAPI spec
gen-api:
    @echo "SKIP gen-api (until P12.T04)"

# ---------------------------------------------------------------- plan / progress

# Regenerate the progress dashboard in Context/PLAN.md
progress:
    python3 Context/plan/progress.py

# List tasks that can be started now
next:
    python3 Context/plan/progress.py --next

# Validate plan task format and dependencies
plan-check:
    python3 Context/plan/progress.py --check

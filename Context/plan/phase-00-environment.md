# Phase 00 — Environment & bootstrap

> **Goal:** a reproducible toolchain and an empty-but-building monorepo with task runner, hooks, CI file, agent entry point and graphify.
> **Depends on:** nothing.
> **Read first:** [TOOLS_TO_USE.md](../TOOLS_TO_USE.md), [CONVENTIONS.md §1–2](CONVENTIONS.md), [PLAN.md §2 shell note](../PLAN.md#2-agent-session-protocol-follow-in-order).
> **Exit criteria:** `mise install && just setup && just check` succeed on a fresh clone. `cargo build`, `pnpm -C web build` and `uv run pytest` (in `validation/`) all pass. The graphify graph exists and its post-commit hook is installed.
> **Machine facts (verified 2026-10-08):**
> - Ubuntu 26.04 on ext4. The user's login shell is **fish**, but the **agent's Bash tool runs bash**.
> - gcc/g++/cmake/make/pkg-config are present. Python 3.14 is present.
> - `uv` is only on PATH via the VS Code snap (`~/snap/code/...`). `graphify` is already installed via **pipx**.
> - **Not installed:** node, rust, just, `gh`.

- [x] **P00.T01** — Verify system prerequisites & commit the plan
  - **Depends:** none
  - **Do:**
    1. Run `for c in git curl gcc g++ make cmake pkg-config; do command -v $c >/dev/null || echo MISSING $c; done`.
    2. Commit the planning files so later `git add -A` doesn't sweep them into an unrelated commit: `git add Context && git commit -m "P00.T01: add implementation plan"`. Check first that `git status` shows nothing else unexpected.
  - **Verify:** No `MISSING` lines; `git log -1` shows the plan commit.
  - **Done when:** All prerequisites are present, or 👤 the user installed the missing ones (`sudo apt install build-essential cmake pkg-config git curl`).
  - **If stuck:** Never run `sudo` yourself. Ask the user.

- [x] **P00.T02** — Install mise and make it visible to fish (user) and bash (agent)
  - **Depends:** P00.T01
  - **Do:**
    1. `curl https://mise.run | sh` (installs `~/.local/bin/mise`).
    2. Ask the user before editing shell configs; show the exact lines. Then:
       - fish (the user's terminal): add `~/.local/bin/mise activate fish | source` to `~/.config/fish/config.fish`.
       - bash (the agent's shell and scripts): add `export PATH="$HOME/.local/share/mise/shims:$HOME/.local/bin:$PATH"` to `~/.bashrc` **and** `~/.profile`. Shims work in non-interactive shells, where `activate` does not.
  - **Verify:** `bash -lc 'command -v mise && mise --version'`; `fish -c 'mise doctor'` reports activated.
  - **Done when:** mise is reachable from both shells.

- [x] **P00.T03** — Pin toolchains in `mise.toml` and `rust-toolchain.toml`
  - **Depends:** P00.T02
  - **Do:**
    1. Create `mise.toml` with `[tools]`: `rust` (latest stable at this date), `node` (current LTS), `pnpm` (latest), `python = "3.13"`, `uv` (latest), `just`, `lefthook`, `gh`, `"cargo:cargo-nextest"`, `"cargo:cargo-insta"`.
    2. Create `rust-toolchain.toml` (`channel` = the same stable version, `components = ["rustfmt", "clippy"]`).
    3. Run `mise trust && mise install`.
    4. Record the exact versions as a new decision (next free number in DECISIONS.md) "Pinned toolchain versions".
  - **Files:** `mise.toml`, `rust-toolchain.toml`, `Context/plan/DECISIONS.md`
  - **Verify:** `bash -lc 'cargo --version; node --version; pnpm --version; just --version; uv --version; gh --version; cargo nextest --version'`, with each resolving to the mise shims (`command -v cargo` → `~/.local/share/mise/shims/cargo`).
  - **Done when:** Versions are pinned and recorded.
  - **If stuck:** `cargo:` backends need a working rust first: install rust, then re-run `mise install`. Or add `cargo-binstall` to mise for faster installs.

- [x] **P00.T04** — Repo skeleton, ignores, editor config, README stub
  - **Depends:** P00.T03
  - **Do:**
    1. Create the folder layout from CONVENTIONS §1 **except `web/` and `docs/`**. Those are created by their scaffolders (T06, P01.T08), which fail or prompt on existing directories. Empty dirs get a `.gitkeep`.
    2. **Extend the existing `.gitignore`.** It is GitHub's Python template, plus `graphify-out/`. Read it first and don't overwrite it. Its bare `lib/`/`lib64/` rules would silently ignore `web/src/lib/`, so anchor them to the root (`/lib/`, `/lib64/`) or remove them, and verify with `git check-ignore -v web/src/lib/x.ts` (must print nothing). Add entries for `target/`, `node_modules/`, `.venv/`, `graphify-out/`, `dist/`, `build/`, `.docusaurus/`, `playwright-report/`, `test-results/`, `*.parquet` outside `validation/goldens/`, and `.env`.
    3. Write `.editorconfig` and `.gitattributes` (`* text=auto eol=lf`).
    4. Write `README.md` with a one-paragraph description and a "Development: see Context/PLAN.md" pointer.
  - **Verify:** `git status` shows only intended files.
  - **Done when:** The layout matches CONVENTIONS §1 (minus web/docs).

- [x] **P00.T05** — Cargo workspace with four crates
  - **Depends:** P00.T04
  - **Do:**
    1. Root `Cargo.toml`: `[workspace]` with `resolver = "3"` and members `crates/*`.
    2. `[workspace.package]` (edition 2024, license MIT unless the user says otherwise, version 0.1.0).
    3. `[workspace.dependencies]` for shared crates (serde, thiserror, anyhow, tracing, approx, proptest, insta).
    4. `[workspace.lints]` with `rust.unsafe_code = "forbid"` and clippy `all = warn`.
    5. Create crates `sim-model` (lib), `sim-core` (lib, depends on sim-model), `sim-api` (lib, depends on sim-core + sim-model), `bldc-sim` (bin, depends on sim-api, clap) whose `main` prints the version.
  - **Files:** `Cargo.toml`, `crates/*/Cargo.toml`, `crates/*/src/lib.rs|main.rs`
  - **Verify:** `cargo build --workspace && cargo clippy --workspace --all-targets -- -D warnings && cargo run -p bldc-sim -- --version && cargo nextest run --workspace --no-tests=pass`
  - **Done when:** Everything builds clean.

- [x] **P00.T06** — pnpm workspace with `web/` (Vite React TS) and Biome
  - **Depends:** P00.T04
  - **Do:**
    1. Root `package.json` (private, `packageManager` pinned to the pnpm version) and `pnpm-workspace.yaml` (`web`, `docs`).
    2. Scaffold **non-interactively** into the not-yet-existing dir: check `pnpm create vite@latest --help` for the version's flags, then e.g. `timeout 300 pnpm create vite@latest web --template react-ts --no-interactive < /dev/null`. Decline any "install and start now" option (it would block).
    3. Enable TS `strict`.
    4. Add Biome at the root (`biome.json` with `"vcs": {"enabled": true, "clientKind": "git", "useIgnoreFile": true}`) and remove ESLint from the template.
    5. Add scripts: `dev`, `build`, `test` (vitest), `check` (biome + tsc --noEmit).
    6. Add Vitest with one trivial test.
  - **Files:** `package.json`, `pnpm-workspace.yaml`, `biome.json`, `web/**`
  - **Verify:** `pnpm install && pnpm -C web build && pnpm -C web test --run && pnpm biome check .`
  - **Done when:** All four pass.
  - **If stuck:** TROUBLESHOOTING §1 (scaffolder hangs).

- [x] **P00.T07** — Python validation project (uv)
  - **Depends:** P00.T04
  - **Do:**
    1. `uv init validation --python 3.13 --no-workspace` (if `validation/` already exists from T04, use `uv init --python 3.13` inside it).
    2. Write `validation/.python-version` = `3.13`.
    3. Add deps: `numpy scipy pyarrow polars matplotlib`; dev deps: `pytest hypothesis ruff`.
    4. Create `validation/tests/test_smoke.py`, which asserts that numpy/scipy import.
    5. Configure ruff in `pyproject.toml`.
  - **Files:** `validation/pyproject.toml`, `validation/uv.lock`, `validation/tests/test_smoke.py`
  - **Verify:** `cd validation && uv sync && uv run python --version` (3.13.x) `&& uv run pytest -q && uv run ruff check .`
  - **Done when:** The smoke test passes on Python 3.13.

- [x] **P00.T08** — `justfile` task runner
  - **Depends:** P00.T05, P00.T06, P00.T07
  - **Do:** Create recipes `setup`, `dev`, `build`, `test`, `test-rust`, `test-web`, `validate`, `e2e`, `fmt`, `lint`, `check-fast`, `check`, `progress`, `next`, `plan-check`, `gen-api` (placeholder until P12), `docs-dev` (placeholder until P01).
    - Set `set shell := ["bash", "-c"]` and export the mise shims on PATH at the top, so recipes work from any shell.
    - `progress` = `python3 Context/plan/progress.py`; `next` = `... --next`; `plan-check` = `... --check`.
    - Rust tests always use `cargo nextest run --workspace --no-tests=pass`.
    - `check-fast` = fmt check + clippy + nextest + `pnpm -C web test --run` + plan-check.
    - `check` = check-fast + validate + e2e + web build + docs build. A recipe whose component doesn't exist yet should print `SKIP (until PNN)` and succeed.
  - **Files:** `justfile`
  - **Verify:** `just --list`; `just check-fast`; `just progress` updates the PLAN.md dashboard.
  - **Done when:** All recipes run.

- [ ] **P00.T09** — lefthook pre-commit hooks
  - **Depends:** P00.T08
  - **Do:** `lefthook.yml` pre-commit jobs, run on staged files where possible:
    - `cargo fmt --check`
    - `pnpm biome check --staged --no-errors-on-unmatched`
    - `uv run --project validation ruff format --check`
    - `just plan-check`

    Then run `lefthook install`.
  - **Files:** `lefthook.yml`
  - **Verify:** Make a deliberately mis-formatted Rust file, stage it, and `git commit` is rejected. Revert.
  - **Done when:** Hooks block bad commits and pass good ones in under ~10 s.

- [ ] **P00.T10** — Agent entry point `CLAUDE.md` (+ `AGENTS.md` symlink)
  - **Depends:** P00.T08
  - **Do:** Write a short root `CLAUDE.md` (≤ 60 lines) with:
    - the project one-liner
    - "Before any work: read `Context/PLAN.md` and follow §2 session protocol"
    - the bash/mise-shims note
    - key `just` commands
    - the ID namespace rule
    - the subagent policy summary (link to PLAN §5)
    - "never use sudo; ask"

    Then `ln -s CLAUDE.md AGENTS.md`.
  - **Files:** `CLAUDE.md`, `AGENTS.md`
  - **Verify:** Read it cold: could a new agent start P00.T11 from it alone?
  - **Done when:** It exists and is concise.

- [ ] **P00.T11** — graphify: first graph and post-commit hook
  - **Depends:** P00.T10
  - **Do:**
    1. `command -v graphify`. It is already installed via pipx; upgrade with `pipx upgrade graphifyy`. Install with `uv tool install graphifyy` **only** if it is missing.
    2. Run the **graphify skill** on the repo root (`/graphify .`), which builds `graphify-out/` (graph.json, GRAPH_REPORT.md, graph.html).
    3. `graphify hook install` (post-commit AST update).
    4. `graphify claude install` (adds a graphify section to CLAUDE.md), then review that section for conflicts with our rules.
  - **Files:** `CLAUDE.md` (graphify section), git hooks
  - **Verify:** `graphify hook status`; `graphify query "what is the plan workflow"` returns nodes; a test commit still runs lefthook **and** graphify.
  - **Done when:** The graph exists and both hooks run. Note in LOG how long the build took.
  - **If stuck:** The skill's extraction subagents are approved (PLAN §5). If lefthook overwrote graphify's `post-commit` (or vice versa), put graphify's command under `post-commit` in `lefthook.yml` and re-run `lefthook install`.

- [ ] **P00.T12** — CI workflow file
  - **Depends:** P00.T08
  - **Do:** Create `.github/workflows/ci.yml` with jobs `rust` (fmt, clippy, nextest), `web` (biome, tsc, vitest, build), `python` (uv sync, ruff, pytest). Use `jdx/mise-action` for toolchains, plus cargo/pnpm/uv caching. Leave commented placeholders for the `e2e`, `validate` and `docs` jobs (enabled in P01/P11/P18).
  - **Files:** `.github/workflows/ci.yml`
  - **Verify:** Lint the YAML (`pnpm dlx @action-validator/cli .github/workflows/ci.yml`).
  - **Done when:** The file is valid. It runs once a remote exists (T13).

- [ ] **P00.T13** — 👤 USER: GitHub remote (optional, non-blocking)
  - **Depends:** P00.T12
  - **Do:** Ask the user whether to create a GitHub repo now.
    - If yes: the user runs `gh auth login` (interactive). After **confirming the name and visibility** with the user, run `gh repo create <name> --private --source . --push`.
    - If no: mark `[-]` with the reason. CI stays dormant until a remote exists.
  - **Verify:** `git remote -v`; the first CI run is green (or the task is skipped).
  - **Done when:** A remote exists with CI green, or the task is skipped by the user.

- [ ] **P00.T14** — Phase gate
  - **Depends:** P00.T01, P00.T02, P00.T03, P00.T04, P00.T05, P00.T06, P00.T07, P00.T08, P00.T09, P00.T10, P00.T11, P00.T12, P00.T13
  - **Do:**
    1. Run the PLAN.md §8 checklist.
    2. Fresh-clone test: `rm -rf /tmp/claude-1000/bldc-clone-test && git clone . /tmp/claude-1000/bldc-clone-test && cd /tmp/claude-1000/bldc-clone-test && mise trust && mise install && just setup && just check-fast`, then delete the clone.
    3. Phase-end review subagent: for this phase the diff range is `$(git rev-list --max-parents=0 HEAD)..HEAD`.
  - **Verify:** Gate checklist complete.
  - **Done when:** Tagged `phase-00-done`.

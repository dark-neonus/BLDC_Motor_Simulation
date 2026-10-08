# Troubleshooting

> General strategy first, then known problems by area. If you solve a new, non-obvious problem, **add it here** (symptom → cause → fix).

## 0. General debugging strategy

1. **Read the first error**, not the last. Compilers cascade.
2. **Check the environment** before the code:
   - `mise doctor` and `mise ls` (are the right tool versions active?)
   - `which cargo node pnpm python` (do they resolve to mise shims, `~/.local/share/mise/...`?)
   - In fish, mise must be activated: `mise activate fish | source` in `~/.config/fish/config.fish`. Without it, the tools "are installed" but not on PATH.
3. **Reproduce minimally:** run the single failing test (`cargo nextest run -p sim-core <name>`, `pnpm -C web vitest run <file>`, `uv run pytest -k <id>`).
4. **Check the version-matched docs.** Rust crates churn (axum, rmcp, mlua, serde-saphyr, diffsol). Open `https://docs.rs/<crate>/<exact version from Cargo.lock>` and the crate's `examples/` for that tag. Don't code from memory of an older API.
5. **Bisect your own change:** `git stash`; does `main` pass? If yes, the problem is in your diff.
6. **Simulation misbehaves?** See §5 before touching the code.
7. After **3 genuinely different attempts**, mark the task `[!]`, log the evidence, and ask the user (PLAN.md §2.6).

## 1. Toolchain & environment

| Symptom | Likely cause | Fix |
|---|---|---|
| `mise: command not found` | mise not installed or not on PATH | `curl https://mise.run \| sh`, then add `~/.local/bin` to PATH and `mise activate fish \| source` to `config.fish`; open a new shell |
| `cargo`/`node` "not found" after `mise install` | Shell not activated | See above; or use `mise exec -- <cmd>` / `mise x -- <cmd>` |
| `error: linker 'cc' not found` | No C toolchain | 👤 ask user: `sudo apt install build-essential` |
| mlua/Luau build fails (`cc1plus`, C++ errors) | g++ missing or too old | 👤 `sudo apt install g++ cmake`. Check the `mlua` features are `["luau", "vendored"]` (Luau is always vendored; don't enable `luajit`/`lua54` too) |
| `failed to run custom build command for openssl-sys` | Some crate pulled native TLS | Prefer `rustls` features (`reqwest` with `default-features = false, features = ["rustls-tls", "json"]`); or 👤 `sudo apt install libssl-dev pkg-config` |
| `pnpm: command not found` | Not pinned/activated | It is pinned in `mise.toml`; `mise install`; or `corepack enable` as a fallback |
| `uv sync` picks Python 3.14 | Missing pin | `validation/.python-version` = `3.13`; `uv python install 3.13` |
| `pyarrow`/`scipy` wheel build from source | Python too new | Ensure 3.13 (D-005) |
| Playwright: "Executable doesn't exist" | Browsers not installed | `pnpm -C web exec playwright install chromium`. System deps: 👤 `sudo pnpm -C web exec playwright install-deps chromium` |
| `graphify: command not found` | Not installed as a uv tool | `uv tool install graphifyy` (two y's); ensure `~/.local/bin` is on PATH |
| Tool "not found" in the agent's Bash but works in the user's terminal | The agent shell is **bash**; the user's is fish | `export PATH="$HOME/.local/share/mise/shims:$HOME/.local/bin:$PATH"`, or `mise x -- <cmd>`. P00.T02 adds the shims to `~/.bashrc`/`~/.profile` |
| `uv` only found under `~/snap/code/...` | It came with the VS Code snap environment | uv is pinned in `mise.toml` (P00.T03), so use the mise one |
| `graphify` already exists | Installed earlier via **pipx** (`~/.local/share/pipx/venvs/graphifyy`) | Use it; upgrade with `pipx upgrade graphifyy`. Don't also `uv tool install` it |
| `gh: command not found` | GitHub CLI not installed | It is pinned in `mise.toml` (`gh`); auth is 👤 interactive: ask the user to run `gh auth login` |
| A scaffolder (`pnpm create vite`, `create docusaurus`) hangs | Interactive prompt (non-empty dir, "install & start now?") | Target a **non-existent** directory, pass the non-interactive flags from `--help` for that version, and run with `< /dev/null` + `timeout 300` so a prompt fails fast instead of hanging |
| `cargo nextest` fails with "no tests to run" | Empty crates early on | Use `cargo nextest run --no-tests=pass` (already in `just` recipes) |
| `biome check --staged` errors about VCS | VCS integration off | `biome.json`: `"vcs": {"enabled": true, "clientKind": "git", "useIgnoreFile": true}` |
| `target/` paths differ | Someone set `CARGO_TARGET_DIR` | Don't set it. Scripts assume `target/release/bldc-sim` (the repo is on ext4, so no need) |

## 2. Rust compile problems

| Symptom | Fix |
|---|---|
| Trait/method "not found" on a crate type | API changed between versions. Check docs.rs for the **locked** version; see the crate's CHANGELOG |
| `uom` type mismatch walls of errors | You mixed quantity kinds; read the first error's two types. At hot-loop boundaries convert with `.get::<si::...>()` (D-003) |
| Orphan/trait-bound errors with nalgebra generics | Use concrete fixed types (`SVector<f64, N>`, `Vector3<f64>`), not generic `N` |
| axum handler "doesn't implement Handler" | Usually an extractor-order problem (body extractor must be last) or a non-`Send` future (holding a `std::sync::MutexGuard` across `.await`). Add `#[axum::debug_handler]` for a readable error |
| rmcp macro errors | Match the macro usage in the rmcp `examples/` at the locked version exactly (it churns). Check the enabled features (`server`, `transport-streamable-http-server`, `macros`) |
| serde-saphyr API unknown | Read its docs.rs page. It mirrors serde_yaml's `from_str`/`to_string` but options differ |
| Workspace dep version conflicts | Declare shared deps once in `[workspace.dependencies]`; run `cargo tree -d` to find duplicates |
| Clippy fails on new lints after a toolchain bump | Fix them (preferred) or `#[allow]` locally with a reason comment; never disable globally |

## 3. Frontend problems

| Symptom | Fix |
|---|---|
| WS connects in prod but not in dev | Vite proxy needs `ws: true` for `/api/stream`. Check `vite.config.ts` |
| CORS errors | In dev, always go through the Vite proxy (same origin). Don't add permissive CORS to the server |
| Types out of sync with the server | `just gen-api` (dump-openapi → openapi-typescript). Never hand-edit `schema.d.ts` |
| Pixi blank canvas in Playwright/headless | Launch Chromium with `--use-gl=angle --use-angle=swiftshader` (or `--enable-unsafe-swiftshader`). For visual tests, assert on exposed state (`window.__bldcViz`), not pixels |
| uPlot not resizing in Dockview panel | Observe panel size with `ResizeObserver` → `plot.setSize()` |
| UI stutters | Check that the render loop isn't driven by React state; batch WS frames; use the decimated stream |
| Tailwind classes not applied | Check the Tailwind v4 config/`@import "tailwindcss"` and that the shadcn CSS variables file is imported once in `main.tsx` |

## 3b. Docs (Docusaurus / MDX) problems

| Symptom | Fix |
|---|---|
| `MDX compilation failed … Could not parse expression with acorn` | Braces in Markdown are JS in MDX. Heading IDs must be `{/* #id */}`, not `{#id}`. Escape literal braces outside math (`\{`). Math inside `$…$`/`$$…$$` is fine (remark-math) |
| MDX fails on a line with `<https://…>` | MDX has no autolinks: write `[https://…](https://…)` |
| `Can't resolve '…/docs/katex/…'` in clientModules | Paths there resolve relative to the site; use `require.resolve()` via `createRequire(import.meta.url)` |
| Broken link/anchor errors at build | `onBrokenLinks`/`onBrokenAnchors` are `throw` on purpose: fix the link, don't relax the setting |

## 4. Python / validation problems

| Symptom | Fix |
|---|---|
| `bldc-sim` not found from pytest | Tests call the binary via the path from env `BLDC_SIM_BIN` (set by `just validate` to the release build). Build first: `cargo build --release -p bldc-sim` |
| Parquet columns missing | Check the CLI output contract (P11.T02). The signals to record are listed in the scenario `record:` section |
| Cross-check mismatch | See §5. Then compare *equations*, not code: list the EQ IDs both sides implement. **Do not look at the Rust code from the refmodel side**; fix the spec if it is ambiguous, then both sides |

## 5. Simulation misbehaving (NaN, explosion, wrong numbers)

1. **Units:** a 1000× error is usually mH vs H, rpm vs rad/s, mm vs m, or poles vs pole pairs.
2. **Step size:** the electrical time constant τ = L/R. RK4 is *stable* up to dt ≈ 2.8·τ, but *accurate* only for dt ≲ τ/20. The authoritative rules are in EQ-NUM and `FidelityConfig` (P03.T06). Halve dt: if the result changes a lot, dt was too large.
3. **Sign conventions:** check against CONVENTIONS §5 (motoring power positive, CCW positive, θ_load = 0 down).
4. **Energy residual:** look at `energy.residual` and which power term jumps. A non-conserving block is the culprit.
5. **Friction stick-slip chatter:** check the Karnopp velocity band (EQ-MECH) is larger than the velocity change per step.
6. **Controller instability vs plant bug:** run the plant open-loop (fixed voltages). If it is sane, the controller gains or sign are wrong.
7. **Compare with the reference model** on the same scenario (`just validate -k <scenario>`), and plot both (`validation/report/plot_compare.py`).
8. **Determinism broken** (two runs differ): a non-seeded RNG, HashMap iteration order in the hot path, or wall-clock time leaking into the sim.

## 6. Runtime / server

| Symptom | Fix |
|---|---|
| `Address already in use :8787` | `ss -ltnp \| grep 8787` → kill the old `bldc-sim`, or `bldc-sim serve --port 8788` |
| Sim/real ratio far below 1 in dev | Debug builds are 10–50× slower. Use `cargo run --release` for performance judgments |
| MCP client can't connect | Check `curl -s localhost:8787/api/health`, then the MCP endpoint path `/mcp` and the transport (streamable HTTP). Test with `npx @modelcontextprotocol/inspector` |
| UI shows stale values after an agent change | The event channel isn't broadcasting that change type. Every mutating API path must emit a `ParamChanged`/`SimEvent` |

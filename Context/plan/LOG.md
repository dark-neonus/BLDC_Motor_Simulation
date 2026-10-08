# Work Log

> Append-only journal. Newest entry at the **bottom**. One entry per session or task batch (format: [CONVENTIONS.md §9](CONVENTIONS.md#9-commits--logs)).
> At session start, read the last 3 entries.

## 2026-10-08 — Planning (agent: Claude)
- Did: polished the idea (POLISHED_IDEA.md), selected tools (TOOLS_TO_USE.md), wrote the plan (PLAN.md + plan/), and recorded initial decisions D-001…D-008.
- Notes: The user approved subagents only for the independent reference model (P11.T01), phase-end reviews and graphify extraction. Ask before any other subagent use. Git: commits directly on `main`.
- Next: P00.T01

## 2026-10-08 — Plan dry-run review (agent: Claude + cold-review subagent)
- Did: a fresh-context subagent dry-ran the whole plan. 10 blockers and ~30 smaller issues were fixed:
  - per-RK-stage module coupling (P03.T02); state events (new P03.T15); energy accumulator moved to P03 (new P03.T16)
  - minimal rotor + interim drive path keeping the app green through P05–P09 (P05.T10/T11, P07.T12)
  - `validate-file` CLI; skeleton FOC params (V_bus 24 V, λ 0.03 Wb); bash vs fish + mise shims; pipx graphify; non-interactive scaffolding; `--no-tests=pass`
  - signal catalog (new P02.T14); URL deep links (new P13.T12); docs search; library-folder API/UI; assert exit codes; dependency-order fixes
  - decisions D-009 (docs embedded) and D-010 (ns rounding)
- Next: P00.T01

## 2026-10-08 — P00.T01–T03 (agent: Claude)
- Did:
  - T01: all prerequisites present; the plan was committed in 92df09c.
  - T02: mise 2026.10.4 installed. With the user's approval, added mise to `~/.config/fish/config.fish` (activate) and to `~/.bashrc` + `~/.profile` (shims on PATH).
  - T03: pinned the toolchain in `mise.toml` + `rust-toolchain.toml` (D-011); `mise install` took ~220 s.
- Verified: `bash -lc` resolves cargo/node/pnpm/just/uv/gh/lefthook/nextest/insta through the mise shims, with the expected versions.
- Notes: the system `tail` is not GNU (rejects `-3`); use `tail -n 3`. Inside the repo, `python` resolves to the mise 3.13.
- Next: P00.T04

## 2026-10-08 — P00.T04–T11 (agent: Claude)
- Did:
  - T04: skeleton dirs. The existing Python-template `.gitignore` was extended, and `lib/`/`lib64/` were anchored to the root (`web/src/lib` is not ignored).
  - T05: Cargo workspace (4 crates).
  - T06: Vite React TS (create-vite 9.2.1, run non-interactively); oxlint replaced by Biome 2.5; Vitest 5.
  - T07: uv project with Python 3.13.16.
  - T08: justfile.
  - T09: lefthook (verified that it rejects a mis-formatted .rs).
  - T10: CLAUDE.md + AGENTS.md symlink.
  - T11: graphify 0.9.80 (pipx, upgraded). Whole-repo graph at root `.` (510 nodes, 755 edges, 58 communities; 13 dangling edges from cross-chunk ids). The Context-only graph backup is in the session scratchpad. Post-commit/post-checkout hooks are installed and the merge driver is registered.
- Notes:
  - `graphify claude install` added a CLAUDE.md section **and** `.claude/settings.json` PreToolUse hooks (graph-query reminders before Bash/Grep/Read). Kept; reported to the user.
  - The doc extraction cost ~216k subagent tokens.
  - The pnpm store is at `/mnt/D/.pnpm-store` (separate filesystem from home).
- Verified: `just check` green; `graphify query` returns nodes; `graphify hook status` shows installed.
- Next: P00.T12

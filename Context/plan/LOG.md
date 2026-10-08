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

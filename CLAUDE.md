# BLDC Motor Simulator — agent instructions

A local simulator and learning lab for brushless PM motors: a Rust physics core and server (REST/WS/MCP), a React web UI, a Docusaurus docs site and a Python validation suite.

## Before any work
1. Read `Context/PLAN.md` and follow its **§2 session protocol** (orient → pick task → verify → close → commit).
2. Read `Context/plan/CONVENTIONS.md` and the last 3 entries of `Context/plan/LOG.md`.
3. Work only on tasks from `Context/plan/phase-*.md`. Mark status in the checkbox and run `just progress`.

## Environment
- The agent shell is **bash**; the user's shell is fish. Tools come from **mise** shims. If a tool is "not found":
  `export PATH="$HOME/.local/share/mise/shims:$HOME/.local/bin:$PATH"` or prefix with `mise x --`.
- **Never use `sudo`.** Ask the user (see TROUBLESHOOTING §1).

## Key commands
| Command | Does |
|---|---|
| `just next` / `just progress` | Startable tasks / regenerate the dashboard |
| `just check-fast` | Gate after every task |
| `just check` | Full gate (phase gates) |
| `just dev` · `just build` · `just validate` · `just e2e` | Run, release build, Python validation, Playwright |

## Non-negotiable rules (full list: PLAN.md §4)
- Physics code cites spec IDs (`EQ-MOT-03`) from `docs/docs/physics/`. Fix the spec first if code and spec disagree.
- Never loosen a test tolerance or delete a failing test to get green.
- **One ID namespace** (CONVENTIONS §3): a parameter or signal path like `motor.electrical.kv` is used identically in YAML, the API, MCP, the help registry and UI `data-agent-id`.
- Deterministic sim: all randomness goes through the seeded RNG service. No panics in the sim loop.
- Commit per task on `main`: `PNN.TMM: summary`. Never `--no-verify`, never force-push.

## Subagents (PLAN.md §5)
Pre-approved: the independent reference model (P11.T01), phase-end reviews (phase gates) and graphify extraction. **Anything else: ask the user first.**

## Troubleshooting
`Context/plan/TROUBLESHOOTING.md`. After 3 different failed approaches, mark the task `[!]`, log it and ask the user.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).

#!/usr/bin/env python3
"""Plan progress tracker (stdlib only).

Scans the task checkboxes in Context/plan/phase-*.md and rewrites the dashboard
block between the PROGRESS markers in Context/PLAN.md.

Usage:
    python3 Context/plan/progress.py           # rewrite dashboard + print summary
    python3 Context/plan/progress.py --next    # print tasks that can be started now
    python3 Context/plan/progress.py --check   # validate task format/deps, exit 1 on problems

Task line format (see PLAN.md §3):
    - [ ] **P03.T02** — Title
      - **Depends:** P03.T01, P02
Status chars: ' ' todo, '~' in progress, 'x' done, '!' blocked, '-' skipped/deferred.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

PLAN_DIR = Path(__file__).resolve().parent
PLAN_MD = PLAN_DIR.parent / "PLAN.md"
START, END = "<!-- PROGRESS:START -->", "<!-- PROGRESS:END -->"

TASK_RE = re.compile(r"^- \[(?P<s>[ x~!\-])\] \*\*(?P<id>P\d{2}\.T\d{2})\*\* — (?P<title>.+?)\s*$")
LOOSE_TASK_RE = re.compile(r"^\s*- \[.?\]\s*\*\*P\d")
DEPS_RE = re.compile(r"^\s+- \*\*Depends:\*\*\s*(?P<deps>.+?)\s*$")
REF_RE = re.compile(r"\bP\d{2}(?:\.T\d{2})?\b")
TITLE_RE = re.compile(r"^# Phase (?P<num>\d{2}) — (?P<title>.+?)\s*$")

ICON = {"todo": "⬜ todo", "progress": "🟧 in progress", "done": "✅ done", "blocked": "⛔ blocked"}


@dataclass
class Task:
    id: str
    title: str
    status: str
    deps: list[str] = field(default_factory=list)
    file: str = ""
    line: int = 0


@dataclass
class Phase:
    num: str
    title: str
    file: str
    tasks: list[Task] = field(default_factory=list)

    @property
    def closed(self) -> int:
        return sum(t.status in "x-" for t in self.tasks)

    @property
    def state(self) -> str:
        if any(t.status == "!" for t in self.tasks):
            return "blocked"
        if self.tasks and self.closed == len(self.tasks):
            return "done"
        if any(t.status in "~x-" for t in self.tasks):
            return "progress"
        return "todo"


def parse() -> tuple[list[Phase], list[str]]:
    problems: list[str] = []
    phases: list[Phase] = []
    for path in sorted(PLAN_DIR.glob("phase-*.md")):
        lines = path.read_text(encoding="utf-8").splitlines()
        m = TITLE_RE.match(lines[0]) if lines else None
        if not m:
            problems.append(f"{path.name}:1 first line must be '# Phase NN — Title'")
            continue
        phase = Phase(m["num"], m["title"], path.name)
        current: Task | None = None
        for i, line in enumerate(lines, 1):
            tm = TASK_RE.match(line)
            if tm:
                current = Task(tm["id"], tm["title"], tm["s"], file=path.name, line=i)
                if not current.id.startswith(f"P{phase.num}."):
                    problems.append(f"{path.name}:{i} task {current.id} is in the wrong phase file")
                phase.tasks.append(current)
                continue
            if LOOSE_TASK_RE.match(line):
                problems.append(f"{path.name}:{i} malformed task line: {line.strip()[:80]}")
                current = None
                continue
            dm = DEPS_RE.match(line)
            if dm and current is not None:
                current.deps = REF_RE.findall(dm["deps"])
        phases.append(phase)
    return phases, problems


def deps_met(task: Task, by_id: dict[str, Task], by_phase: dict[str, Phase]) -> bool:
    for ref in task.deps:
        if "." in ref:
            dep = by_id.get(ref)
            if dep is None or dep.status not in "x-":
                return False
        else:
            ph = by_phase.get(ref[1:])
            if ph is None or ph.state != "done":
                return False
    return True


def check(phases: list[Phase], problems: list[str]) -> list[str]:
    seen: dict[str, Task] = {}
    nums = {p.num for p in phases}
    for p in phases:
        for t in p.tasks:
            if t.id in seen:
                problems.append(f"{t.file}:{t.line} duplicate id {t.id} (also {seen[t.id].file}:{seen[t.id].line})")
            seen[t.id] = t
    for p in phases:
        for t in p.tasks:
            for ref in t.deps:
                if ("." in ref and ref not in seen) or ("." not in ref and ref[1:] not in nums):
                    problems.append(f"{t.file}:{t.line} {t.id} depends on unknown {ref}")
    return problems


def bar(done: int, total: int, width: int) -> str:
    filled = round(width * done / total) if total else 0
    return "█" * filled + "░" * (width - filled)


def available(phases: list[Phase]) -> list[Task]:
    by_id = {t.id: t for p in phases for t in p.tasks}
    by_phase = {p.num: p for p in phases}
    return [t for p in phases for t in p.tasks if t.status == " " and deps_met(t, by_id, by_phase)]


def render(phases: list[Phase]) -> str:
    total = sum(len(p.tasks) for p in phases)
    closed = sum(p.closed for p in phases)
    pct = 100 * closed // total if total else 0
    out = [
        "_Auto-generated by `just progress` (or `python3 Context/plan/progress.py`). Do not edit by hand._",
        "",
        f"**Overall: {closed} / {total} tasks ({pct}%)** `{bar(closed, total, 30)}`",
        "",
        "| Phase | Title | Done | Progress | State |",
        "|---|---|---|---|---|",
    ]
    for p in phases:
        out.append(
            f"| [P{p.num}](plan/{p.file}) | {p.title} | {p.closed}/{len(p.tasks)} "
            f"| `{bar(p.closed, len(p.tasks), 10)}` | {ICON[p.state]} |"
        )
    tasks = [t for p in phases for t in p.tasks]
    for label, status in (("In progress", "~"), ("Blocked", "!")):
        items = [t for t in tasks if t.status == status]
        out += ["", f"**{label}:** " + (", ".join(f"`{t.id}` {t.title}" for t in items) if items else "none")]
    nxt = available(phases)[:5]
    out += ["", "**Next available:** " + (", ".join(f"`{t.id}` {t.title}" for t in nxt) if nxt else "none")]
    return "\n".join(out)


def main() -> int:
    phases, problems = parse()
    if "--check" in sys.argv:
        problems = check(phases, problems)
        for pr in problems:
            print("PLAN CHECK:", pr)
        print("plan check OK" if not problems else f"{len(problems)} problem(s)")
        return 1 if problems else 0
    if "--next" in sys.argv:
        for t in available(phases):
            print(f"{t.id}  {t.title}  ({t.file}:{t.line})")
        return 0
    text = PLAN_MD.read_text(encoding="utf-8")
    if START not in text or END not in text:
        print(f"PLAN.md is missing the {START} / {END} markers", file=sys.stderr)
        return 1
    head, rest = text.split(START, 1)
    _, tail = rest.split(END, 1)
    PLAN_MD.write_text(f"{head}{START}\n{render(phases)}\n{END}{tail}", encoding="utf-8")
    total = sum(len(p.tasks) for p in phases)
    print(f"dashboard updated: {sum(p.closed for p in phases)}/{total} tasks closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

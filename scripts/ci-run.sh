#!/usr/bin/env bash
# Run a CI step; on failure, publish the last lines of its output as a GitHub
# annotation (readable without login via the check-runs API).
# Usage: scripts/ci-run.sh <label> <command...>
set -uo pipefail
label="$1"; shift
log="$(mktemp)"
"$@" 2>&1 | tee "$log"
status=${PIPESTATUS[0]}
if [ "$status" -ne 0 ]; then
  msg="$(tail -n 40 "$log" | sed 's/%/%25/g' | tr '\n' '|' | sed 's/|/%0A/g' | cut -c1-3500)"
  echo "::error title=${label} failed (exit ${status})::${msg}"
fi
exit "$status"

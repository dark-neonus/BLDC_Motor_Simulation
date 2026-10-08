"""Shared fixtures: locate the bldc-sim binary and run scenarios through the CLI.

The binary path comes from env BLDC_SIM_BIN (set by `just validate`), defaulting to the
release build. Outputs follow the CLI contract: signals.parquet + meta.json.
"""

import os
import subprocess
from pathlib import Path

import polars as pl
import pytest

REPO = Path(__file__).resolve().parents[2]
SCENARIOS = REPO / "validation" / "scenarios"


def _binary() -> Path:
    return Path(os.environ.get("BLDC_SIM_BIN", REPO / "target" / "release" / "bldc-sim"))


@pytest.fixture(scope="session")
def bldc_sim() -> Path:
    path = _binary()
    if not path.exists():
        pytest.fail(f"bldc-sim binary not found at {path}; run `cargo build --release -p bldc-sim`")
    return path


@pytest.fixture
def run_scenario(bldc_sim: Path, tmp_path: Path):
    """Run `bldc-sim run-scenario <name>.yaml` and return the recorded signals."""

    def _run(name: str) -> pl.DataFrame:
        out = tmp_path / name
        subprocess.run(
            [str(bldc_sim), "run-scenario", str(SCENARIOS / f"{name}.yaml"), "--out", str(out)],
            check=True,
            capture_output=True,
        )
        return pl.read_parquet(out / "signals.parquet")

    return _run

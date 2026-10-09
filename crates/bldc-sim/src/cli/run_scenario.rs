//! `bldc-sim run-scenario <file.yaml> --out <dir>`: batch run → signals.parquet,
//! signals.csv, meta.json and (full scenarios) asserts.json. This output contract is
//! consumed by the Python validation suite (validation/tests/conftest.py).
//!
//! Two formats: full scenarios (P04.T11, recognised by `timeline:`) and the legacy
//! skeleton scenarios used by the P01 validation tests.

use std::fs;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use arrow_array::{ArrayRef, Float64Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema};
use sha2::{Digest, Sha256};
use sim_core::build::build_engine;
use sim_core::scenario_run::AssertResult;
use sim_core::skeleton::scenario::{COLUMNS, SkeletonScenario, simulate};
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scenario::Scenario;

/// Runs the scenario; returns false if any assert failed.
pub fn run(file: &Path, out: &Path, asserts: bool) -> Result<bool> {
    let text = fs::read_to_string(file).with_context(|| format!("reading {}", file.display()))?;
    let name = file.display().to_string();
    let full = text.lines().any(|l| l.starts_with("timeline:"));
    let (columns, rows, seed, results, warnings) = if full {
        let sc: Scenario = parse_yaml(&text, &name)?;
        let lib = Library::open_default()?;
        let scene = sc.scene.resolve(&lib, "scene")?.resolve(&lib)?;
        let built = build_engine(&scene)?;
        let o = sim_core::scenario_run::run(&sc, built, asserts).map_err(anyhow::Error::msg)?;
        fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;
        for (n, bytes) in &o.snapshots {
            fs::write(out.join(format!("{n}.snapshot")), bytes)?;
        }
        (o.columns, o.rows, scene.seed, Some(o.asserts), o.warnings)
    } else {
        let sc: SkeletonScenario =
            serde_saphyr::from_str(&text).with_context(|| format!("parsing {}", file.display()))?;
        let rows = simulate(&sc).map_err(anyhow::Error::msg)?;
        let cols = COLUMNS.iter().map(|c| (*c).to_owned()).collect();
        (
            cols,
            rows.iter().map(|r| r.to_vec()).collect(),
            0,
            None,
            vec![],
        )
    };
    for w in &warnings {
        eprintln!("warning: {w}");
    }
    fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;
    write_parquet(&out.join("signals.parquet"), &columns, &rows)?;
    write_csv(&out.join("signals.csv"), &columns, &rows)?;
    let meta = serde_json::json!({
        "bldc_sim_version": env!("CARGO_PKG_VERSION"),
        "scenario_file": name,
        "scenario_sha256": hex(&Sha256::digest(text.as_bytes())),
        "seed": seed,
        "rows": rows.len(),
        "columns": columns,
        "warnings": warnings,
    });
    fs::write(out.join("meta.json"), serde_json::to_string_pretty(&meta)?)?;
    println!("wrote {} rows to {}", rows.len(), out.display());
    let Some(results) = results else {
        return Ok(true);
    };
    fs::write(
        out.join("asserts.json"),
        serde_json::to_string_pretty(&results)?,
    )?;
    print_table(&results);
    Ok(results.iter().all(|r| r.passed))
}

fn print_table(rs: &[AssertResult]) {
    if rs.is_empty() {
        return;
    }
    println!(
        "\n{:<6} {:<22} {:<28} {:>14} {:>10}",
        "result", "signal", "condition", "observed", "at [s]"
    );
    for r in rs {
        let cond = format!("{:?} {} ± {}", r.op, r.value, r.tol).to_lowercase();
        let res = if r.passed { "PASS" } else { "FAIL" };
        println!(
            "{res:<6} {:<22} {cond:<28} {:>14.6} {:>10.4}",
            r.signal, r.observed, r.t_observed
        );
        if !r.passed {
            for m in [&r.message, &r.note].into_iter().flatten() {
                println!("       ↳ {m}");
            }
        }
    }
    let failed = rs.iter().filter(|r| !r.passed).count();
    println!("\n{} passed, {failed} failed", rs.len() - failed);
}

fn write_parquet(path: &Path, columns: &[String], rows: &[Vec<f64>]) -> Result<()> {
    let fields: Vec<Field> = columns
        .iter()
        .map(|c| Field::new(c, DataType::Float64, false))
        .collect();
    let schema = Arc::new(Schema::new(fields));
    let cols: Vec<ArrayRef> = (0..columns.len())
        .map(|j| Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r[j]))) as ArrayRef)
        .collect();
    let batch = RecordBatch::try_new(Arc::clone(&schema), cols)?;
    let file = fs::File::create(path).with_context(|| format!("creating {}", path.display()))?;
    let mut writer = parquet::arrow::ArrowWriter::try_new(file, schema, None)?;
    writer.write(&batch)?;
    writer.close()?;
    Ok(())
}

fn write_csv(path: &Path, columns: &[String], rows: &[Vec<f64>]) -> Result<()> {
    let mut s = columns.join(",");
    s.push('\n');
    for r in rows {
        let line: Vec<String> = r.iter().map(|v| format!("{v:e}")).collect();
        s.push_str(&line.join(","));
        s.push('\n');
    }
    fs::write(path, s).with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

/// Lower-case hex encoding (sha2 0.11 digests don't implement `LowerHex`).
fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

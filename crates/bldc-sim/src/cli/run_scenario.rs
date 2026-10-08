//! `bldc-sim run-scenario <file.yaml> --out <dir>`: batch run → signals.parquet,
//! signals.csv and meta.json. This output contract is consumed by the Python
//! validation suite (validation/tests/conftest.py).

use std::fs;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use arrow_array::{ArrayRef, Float64Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema};
use sha2::{Digest, Sha256};
use sim_core::skeleton::scenario::{COLUMNS, SkeletonScenario, simulate};

pub fn run(file: &Path, out: &Path) -> Result<()> {
    let text = fs::read_to_string(file).with_context(|| format!("reading {}", file.display()))?;
    let sc: SkeletonScenario =
        serde_saphyr::from_str(&text).with_context(|| format!("parsing {}", file.display()))?;
    let rows = simulate(&sc).map_err(anyhow::Error::msg)?;
    fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;

    write_parquet(&out.join("signals.parquet"), &rows)?;
    write_csv(&out.join("signals.csv"), &rows)?;
    let meta = serde_json::json!({
        "bldc_sim_version": env!("CARGO_PKG_VERSION"),
        "scenario_file": file.display().to_string(),
        "scenario_sha256": hex(&Sha256::digest(text.as_bytes())),
        "seed": 0,
        "rows": rows.len(),
        "columns": COLUMNS,
    });
    fs::write(out.join("meta.json"), serde_json::to_string_pretty(&meta)?)?;
    println!("wrote {} rows to {}", rows.len(), out.display());
    Ok(())
}

fn write_parquet(path: &Path, rows: &[[f64; 6]]) -> Result<()> {
    let fields: Vec<Field> = COLUMNS
        .iter()
        .map(|c| Field::new(*c, DataType::Float64, false))
        .collect();
    let schema = Arc::new(Schema::new(fields));
    let columns: Vec<ArrayRef> = (0..COLUMNS.len())
        .map(|j| Arc::new(Float64Array::from_iter_values(rows.iter().map(|r| r[j]))) as ArrayRef)
        .collect();
    let batch = RecordBatch::try_new(Arc::clone(&schema), columns)?;
    let file = fs::File::create(path).with_context(|| format!("creating {}", path.display()))?;
    let mut writer = parquet::arrow::ArrowWriter::try_new(file, schema, None)?;
    writer.write(&batch)?;
    writer.close()?;
    Ok(())
}

fn write_csv(path: &Path, rows: &[[f64; 6]]) -> Result<()> {
    let mut s = COLUMNS.join(",");
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

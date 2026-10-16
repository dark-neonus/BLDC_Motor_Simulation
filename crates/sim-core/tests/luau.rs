//! P09.T10: the sandboxed Luau custom block — V-CTRL-010 (a Luau PI equals the Rust PI
//! to 1e-12), the instruction budget and memory limit turn runaway scripts into errors,
//! errors carry the script line, and the state goes into snapshots.

use std::time::Instant;

use sim_core::control::luau_block::{DEFAULT_BUDGET, DEFAULT_MEMORY, LuauBlock, LuauConfig};
use sim_core::control::pid::{AntiWindup, Pid};
use sim_core::engine::block::{DiscreteBlock, StepCtx};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;

const PI_SCRIPT: &str = r#"
function init(p)
  return { i = 0, kp = p.kp, ki = p.ki, lo = p.lo, hi = p.hi }
end

function step(inp, s, dt)
  local e = inp["x.r"] - inp["x.y"]
  local us = s.kp * e + s.i
  local u = math.clamp(us, s.lo, s.hi)
  local pushes = (us > s.hi and e * s.ki > 0) or (us < s.lo and e * s.ki < 0)
  if not pushes then
    s.i = s.i + dt * s.ki * e
  end
  return { ["x.u"] = u }
end
"#;

const T: f64 = 50e-6;

struct Rig {
    bus: SignalBus,
    r: SignalId,
    y: SignalId,
    u: SignalId,
}

fn rig() -> Rig {
    let mut bus = SignalBus::new();
    let mut reg = |p: &str| bus.register(p, "-", "", SignalKind::Input).unwrap();
    let (r, y, u) = (reg("x.r"), reg("x.y"), reg("x.u"));
    Rig { bus, r, y, u }
}

fn block(script: &str, rig: &Rig, budget: u64, memory: usize) -> Result<LuauBlock, String> {
    LuauBlock::new(
        LuauConfig {
            script: script.into(),
            period: SimTime::from_nanos(50_000),
            inputs: vec!["x.r".into(), "x.y".into()],
            outputs: vec!["x.u".into()],
            params: vec![
                ("kp".into(), 0.7),
                ("ki".into(), 120.0),
                ("lo".into(), -2.0),
                ("hi".into(), 2.0),
            ],
            memory,
            budget,
        },
        &rig.bus,
    )
}

fn step(b: &mut LuauBlock, bus: &mut SignalBus, k: i64) -> Result<(), String> {
    let mut ctx = StepCtx {
        t: SimTime::from_nanos(50_000 * k),
        bus,
    };
    b.step(&mut ctx).map_err(|e| e.to_string())
}

/// V-CTRL-010: the same PI (clamping anti-windup) in Luau and in Rust on the same
/// inputs, through saturation and back: identical outputs (1e-12).
#[test]
fn v_ctrl_010_luau_pi_matches_the_rust_pi() {
    let mut rg = rig();
    let mut b = block(PI_SCRIPT, &rg, DEFAULT_BUDGET, DEFAULT_MEMORY).unwrap();
    let mut pi = Pid::pi(0.7, 120.0, (-2.0, 2.0), AntiWindup::Clamping);
    let mut y = 0.0;
    for k in 0..20_000 {
        let r = if k < 10_000 { 5.0 } else { -1.0 } * (1.0 + 0.1 * (k as f64 * 1e-3).sin());
        rg.bus.set(rg.r, r);
        rg.bus.set(rg.y, y);
        step(&mut b, &mut rg.bus, k).unwrap();
        let want = pi.update(r, y, T);
        let got = rg.bus.get(rg.u);
        assert!((got - want).abs() <= 1e-12, "k {k}: {got} vs {want}");
        y += T * 40.0 * (got - 0.2 * y); // a first-order plant
    }
}

#[test]
fn an_endless_loop_is_interrupted_with_an_error() {
    let mut rg = rig();
    let script =
        "function init(p) return {} end\nfunction step(i, s, dt)\n  while true do end\nend\n";
    let mut b = block(script, &rg, 100_000, DEFAULT_MEMORY).unwrap();
    let start = Instant::now();
    let err = step(&mut b, &mut rg.bus, 0).unwrap_err();
    assert!(err.contains("budget"), "{err}");
    assert!(start.elapsed().as_secs_f64() < 5.0, "interrupted promptly");
}

#[test]
fn a_memory_bomb_is_an_error() {
    let mut rg = rig();
    let script = "function init(p) return {} end\nfunction step(i, s, dt)\n  local x = string.rep(\"x\", 1e9)\n  return { [\"x.u\"] = #x }\nend\n";
    let mut b = block(script, &rg, DEFAULT_BUDGET, 8 << 20).unwrap();
    let err = step(&mut b, &mut rg.bus, 0).unwrap_err();
    assert!(err.to_lowercase().contains("memory"), "{err}");
}

#[test]
fn errors_show_the_script_line() {
    let mut rg = rig();
    let script = "function init(p) return {} end\nfunction step(i, s, dt)\n  return nil + 1\nend\n";
    let mut b = block(script, &rg, DEFAULT_BUDGET, DEFAULT_MEMORY).unwrap();
    let err = step(&mut b, &mut rg.bus, 0).unwrap_err();
    assert!(err.contains("custom:3:"), "{err}");
    // A syntax error is reported at build, with its line.
    let err = block(
        "function init(p)\n  return {\nend\n",
        &rg,
        DEFAULT_BUDGET,
        DEFAULT_MEMORY,
    )
    .err()
    .unwrap();
    assert!(err.contains("custom:"), "{err}");
    // Missing functions are named.
    let err = block("x = 1\n", &rg, DEFAULT_BUDGET, DEFAULT_MEMORY)
        .err()
        .unwrap();
    assert!(err.contains("function init"), "{err}");
}

#[test]
fn state_goes_into_snapshots() {
    let mut rg = rig();
    let mut a = block(PI_SCRIPT, &rg, DEFAULT_BUDGET, DEFAULT_MEMORY).unwrap();
    for k in 0..100 {
        rg.bus.set(rg.r, 1.0);
        step(&mut a, &mut rg.bus, k).unwrap();
    }
    let saved = a.save();
    let mut b = block(PI_SCRIPT, &rg, DEFAULT_BUDGET, DEFAULT_MEMORY).unwrap();
    b.restore(&saved).unwrap();
    step(&mut a, &mut rg.bus, 100).unwrap();
    let ua = rg.bus.get(rg.u);
    step(&mut b, &mut rg.bus, 100).unwrap();
    assert_eq!(rg.bus.get(rg.u), ua);
    // A function in the state cannot be saved: the snapshot says so.
    let script = "function init(p) return { f = function() end } end\nfunction step(i, s, dt) return {} end\n";
    let c = block(script, &rg, DEFAULT_BUDGET, DEFAULT_MEMORY).unwrap();
    assert_eq!(c.save()["unsaved"], true);
}

/// The PI script at 20 kHz: the cost per step is logged (the real-time requirement is
/// < 50 µs; the release-build number is in the LOG and the criterion bench).
#[test]
fn luau_pi_step_cost_is_logged() {
    let mut rg = rig();
    let mut b = block(PI_SCRIPT, &rg, DEFAULT_BUDGET, DEFAULT_MEMORY).unwrap();
    let n = 20_000;
    let start = Instant::now();
    for k in 0..n {
        rg.bus.set(rg.r, 1.0);
        step(&mut b, &mut rg.bus, k).unwrap();
    }
    let us = start.elapsed().as_secs_f64() / n as f64 * 1e6;
    println!(
        "luau PI step: {us:.2} µs ({} build)",
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        }
    );
    assert!(us < 50.0, "{us} µs per step exceeds the 20 kHz budget");
}

/// `family: custom` in a scene: a Luau velocity PI at 20 kHz writes `ctrl.foc.iq_ref`
/// on top of the current loop and holds the speed setpoint.
#[test]
fn custom_family_scene_runs_a_luau_speed_loop() {
    use sim_core::engine::commands::{ChangeSource, EngineCommand};
    use sim_model::io::parse_yaml;
    use sim_model::library::Library;
    use sim_model::scene::Scene;
    let dir = std::env::temp_dir().join(format!("bldc-luau-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let script = r#"
function init(p)
  -- EQ-CTRL-04 design at 20 Hz on the motor-side inertia.
  local wv = 2 * math.pi * 20
  local kp = wv * p.j / p.kt
  return { i = 0, kp = kp, ki = kp * wv / 4, lim = p.i_max }
end

function step(inp, s, dt)
  local e = inp["ctrl.omega_ref"] - inp["motor.omega"]
  local us = s.kp * e + s.i
  local u = math.clamp(us, -s.lim, s.lim)
  if u == us then s.i = s.i + dt * s.ki * e end
  return { ["ctrl.foc.iq_ref"] = u }
end
"#;
    let yaml = format!(
        "schema_version: 1\nname: s\n\
         motor: {{ preset: \"builtin:motors/4108-outrunner\" }}\n\
         inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         controller:\n  family: custom\n  rate: 20 kHz\n  inputs: [ctrl.omega_ref, motor.omega]\n  outputs: [ctrl.foc.iq_ref]\n  limits: {{ current: 4 A }}\n  script: |\n{}\n",
        script
            .lines()
            .map(|l| format!("    {l}"))
            .collect::<Vec<_>>()
            .join("\n")
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    let mut e = sim_core::build::build_engine(&sc.resolve(&lib).unwrap())
        .unwrap()
        .engine;
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 30.0,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    let w = e.bus.get(e.bus.id("motor.omega").unwrap());
    assert!((w / 30.0 - 1.0).abs() < 0.02, "ω {w}");
}

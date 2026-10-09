//! Validation of the non-motor components (P04 gate review): unit kinds, signs and
//! ranges, so that a file which parses is also physically usable by `build_engine`.

use crate::constraints::{Issue, Severity, issue, si};
use crate::param::Param;
use crate::params::*;
use crate::units::Kind;

#[derive(Clone, Copy, PartialEq)]
enum Range {
    Any,
    NonNeg,
    Pos,
}

struct V {
    out: Vec<Issue>,
}

impl V {
    fn p(&mut self, p: &Param, kind: Kind, path: &str, r: Range) -> Option<f64> {
        let v = si(p, kind, path, &mut self.out)?;
        let bad = match r {
            Range::Any => false,
            Range::NonNeg => v < 0.0,
            Range::Pos => v <= 0.0,
        };
        if bad {
            let what = if r == Range::Pos {
                "greater than zero"
            } else {
                "zero or more"
            };
            self.out
                .push(issue(Severity::Reject, path, format!("must be {what}")));
            return None;
        }
        Some(v)
    }
    fn o(&mut self, p: &Option<Param>, kind: Kind, path: &str, r: Range) -> Option<f64> {
        p.as_ref().and_then(|p| self.p(p, kind, path, r))
    }
    fn reject(&mut self, path: &str, msg: impl Into<String>) {
        self.out.push(issue(Severity::Reject, path, msg));
    }
    fn warn(&mut self, path: &str, msg: impl Into<String>) {
        self.out.push(issue(Severity::Warn, path, msg));
    }
}

use Range::{Any, NonNeg, Pos};

pub fn validate_gearbox(g: &GearboxParams) -> Vec<Issue> {
    let mut v = V { out: vec![] };
    if !(g.ratio.is_finite() && g.ratio > 0.0) {
        v.reject(
            "gearbox.ratio",
            "ratio must be greater than zero (motor turns per output turn)",
        );
    }
    if !(g.efficiency > 0.0 && g.efficiency <= 1.0) {
        v.reject("gearbox.efficiency", "efficiency must be in (0, 1]");
    }
    v.o(&g.backlash, Kind::Angle, "gearbox.backlash", NonNeg);
    v.o(
        &g.damping,
        Kind::RotationalDamping,
        "gearbox.damping",
        NonNeg,
    );
    v.o(&g.j_in, Kind::Inertia, "gearbox.j_in", NonNeg);
    v.o(&g.j_out, Kind::Inertia, "gearbox.j_out", NonNeg);
    v.out
}

fn friction(v: &mut V, f: &Friction, at: &str) {
    let ts = v.p(
        &f.static_torque,
        Kind::Torque,
        &format!("{at}.static_torque"),
        NonNeg,
    );
    let tc = v.p(&f.coulomb, Kind::Torque, &format!("{at}.coulomb"), NonNeg);
    v.p(
        &f.viscous,
        Kind::RotationalDamping,
        &format!("{at}.viscous"),
        NonNeg,
    );
    if let (Some(ts), Some(tc)) = (ts, tc)
        && tc > ts
    {
        v.reject(
            &format!("{at}.coulomb"),
            "Coulomb friction cannot exceed static friction (EQ-MECH-05)",
        );
    }
}

pub fn validate_load(l: &LoadParams) -> Vec<Issue> {
    let mut v = V { out: vec![] };
    match l {
        LoadParams::Arm {
            arm_length,
            arm_mass,
            mass,
            distance,
            friction: f,
            ..
        } => {
            v.p(arm_length, Kind::Length, "load.arm_length", Pos);
            v.p(arm_mass, Kind::Mass, "load.arm_mass", NonNeg);
            v.p(mass, Kind::Mass, "load.mass", NonNeg);
            v.p(distance, Kind::Length, "load.distance", NonNeg);
            if let Some(f) = f {
                friction(&mut v, f, "load.friction");
            }
        }
        LoadParams::Constant { torque } => {
            v.p(torque, Kind::Torque, "load.torque", Any);
        }
        LoadParams::Brake { torque } => {
            v.p(torque, Kind::Torque, "load.torque", NonNeg);
        }
        LoadParams::Viscous { coefficient } => {
            v.p(
                coefficient,
                Kind::RotationalDamping,
                "load.coefficient",
                NonNeg,
            );
        }
        LoadParams::Flywheel { inertia } => {
            v.p(inertia, Kind::Inertia, "load.inertia", Pos);
        }
    }
    v.out
}

pub fn validate_inverter(i: &InverterParams) -> Vec<Issue> {
    let mut v = V { out: vec![] };
    let f = v.p(&i.f_pwm, Kind::Frequency, "inverter.f_pwm", Pos);
    let dt = v.p(&i.dead_time, Kind::Time, "inverter.dead_time", NonNeg);
    v.p(&i.r_on, Kind::Resistance, "inverter.r_on", NonNeg);
    v.p(&i.v_f, Kind::Voltage, "inverter.v_f", NonNeg);
    v.p(&i.t_rise, Kind::Time, "inverter.t_rise", NonNeg);
    v.p(&i.t_fall, Kind::Time, "inverter.t_fall", NonNeg);
    v.o(&i.i_eps, Kind::Current, "inverter.i_eps", Pos);
    v.o(&i.v_max, Kind::Voltage, "inverter.v_max", Pos);
    v.o(&i.i_max, Kind::Current, "inverter.i_max", Pos);
    if let (Some(f), Some(dt)) = (f, dt)
        && dt * f > 0.1
    {
        v.warn(
            "inverter.dead_time",
            "dead time is more than 10 % of the PWM period",
        );
    }
    v.out
}

pub fn validate_supply(s: &SupplyParams) -> Vec<Issue> {
    let mut v = V { out: vec![] };
    match s {
        SupplyParams::Ideal { voltage } => {
            v.p(voltage, Kind::Voltage, "supply.voltage", Pos);
        }
        SupplyParams::Psu {
            voltage,
            current_limit,
            r_out,
        } => {
            v.p(voltage, Kind::Voltage, "supply.voltage", Pos);
            v.p(current_limit, Kind::Current, "supply.current_limit", Pos);
            v.p(r_out, Kind::Resistance, "supply.r_out", NonNeg);
        }
        SupplyParams::Battery {
            series,
            parallel,
            capacity_cell,
            r0_cell,
            r1_cell,
            c1_cell,
            soc_init,
            ocv_cell,
            ..
        } => {
            if *series == 0 || *parallel == 0 {
                v.reject(
                    "supply.series",
                    "series and parallel cell counts must be at least 1",
                );
            }
            v.p(capacity_cell, Kind::Charge, "supply.capacity_cell", Pos);
            v.p(r0_cell, Kind::Resistance, "supply.r0_cell", NonNeg);
            v.p(r1_cell, Kind::Resistance, "supply.r1_cell", NonNeg);
            v.p(c1_cell, Kind::Capacitance, "supply.c1_cell", Pos);
            if !(0.0..=1.0).contains(soc_init) {
                v.reject("supply.soc_init", "state of charge must be in [0, 1]");
            }
            let ascending = ocv_cell.windows(2).all(|w| w[1].0 > w[0].0);
            let in_range = ocv_cell
                .iter()
                .all(|(x, y)| (0.0..=1.0).contains(x) && *y > 0.0);
            if ocv_cell.len() < 2 || !ascending || !in_range {
                v.reject(
                    "supply.ocv_cell",
                    "OCV table needs ≥ 2 points with strictly ascending SoC in [0, 1] and positive volts",
                );
            }
        }
    }
    v.out
}

fn common(v: &mut V, c: &SensorCommon, at: &str) {
    v.o(
        &c.update_rate,
        Kind::Frequency,
        &format!("{at}.update_rate"),
        Pos,
    );
    v.o(&c.latency, Kind::Time, &format!("{at}.latency"), NonNeg);
}

pub fn validate_sensors(s: &SensorsParams) -> Vec<Issue> {
    let mut v = V { out: vec![] };
    if let Some(h) = &s.hall {
        for (i, o) in h.offsets.iter().enumerate() {
            v.p(o, Kind::Angle, &format!("sensors.hall.offsets.{i}"), Any);
        }
        if h.offsets.len() > 3 {
            v.reject("sensors.hall.offsets", "a Hall set has three sensors");
        }
        common(&mut v, &h.common, "sensors.hall");
    }
    match &s.encoder {
        Some(EncoderParams::Magnetic {
            inl,
            eccentricity,
            air_gap,
            gap_min,
            gap_max,
            mount_offset,
            common: c,
            ..
        }) => {
            v.o(inl, Kind::Angle, "sensors.encoder.inl", NonNeg);
            v.o(
                eccentricity,
                Kind::Length,
                "sensors.encoder.eccentricity",
                NonNeg,
            );
            v.o(air_gap, Kind::Length, "sensors.encoder.air_gap", Pos);
            v.o(gap_min, Kind::Length, "sensors.encoder.gap_min", Pos);
            v.o(gap_max, Kind::Length, "sensors.encoder.gap_max", Pos);
            v.o(
                mount_offset,
                Kind::Angle,
                "sensors.encoder.mount_offset",
                Any,
            );
            common(&mut v, c, "sensors.encoder");
        }
        Some(EncoderParams::Incremental {
            cpr,
            index_angle,
            common: c,
        }) => {
            if *cpr == 0 {
                v.reject(
                    "sensors.encoder.cpr",
                    "counts per revolution must be at least 1",
                );
            }
            v.o(index_angle, Kind::Angle, "sensors.encoder.index_angle", Any);
            common(&mut v, c, "sensors.encoder");
        }
        None => {}
    }
    if let Some(a) = &s.adc {
        v.p(&a.full_scale, Kind::Current, "sensors.adc.full_scale", Pos);
        v.o(&a.offset, Kind::Current, "sensors.adc.offset", Any);
        if !(2..=3).contains(&a.shunts) {
            v.reject("sensors.adc.shunts", "current sensing uses 2 or 3 shunts");
        }
        common(&mut v, &a.common, "sensors.adc");
    }
    v.out
}

fn loop_(v: &mut V, l: &LoopParams, at: &str) {
    let rate = v.p(&l.rate, Kind::Frequency, &format!("{at}.rate"), Pos);
    let bw = v.o(
        &l.bandwidth,
        Kind::Frequency,
        &format!("{at}.bandwidth"),
        Pos,
    );
    if let (Some(r), Some(b)) = (rate, bw)
        && b > r / 5.0
    {
        v.warn(
            &format!("{at}.bandwidth"),
            "bandwidth above rate / 5 makes a discrete loop poorly damped",
        );
    }
}

fn limits(v: &mut V, l: &Limits) {
    v.p(&l.current, Kind::Current, "controller.limits.current", Pos);
    v.o(
        &l.speed,
        Kind::AngularVelocity,
        "controller.limits.speed",
        Pos,
    );
    if let Some(f) = l.voltage_fraction
        && !(f > 0.0 && f <= 1.0)
    {
        v.reject(
            "controller.limits.voltage_fraction",
            "voltage fraction must be in (0, 1]",
        );
    }
}

pub fn validate_controller(c: &ControllerParams) -> Vec<Issue> {
    let mut v = V { out: vec![] };
    match c {
        ControllerParams::SixStep {
            rate,
            speed_loop,
            advance,
            limits: l,
        } => {
            v.p(rate, Kind::Frequency, "controller.rate", Pos);
            if let Some(s) = speed_loop {
                loop_(&mut v, s, "controller.speed_loop");
            }
            v.o(advance, Kind::Angle, "controller.advance", Any);
            limits(&mut v, l);
        }
        ControllerParams::OpenLoop {
            rate,
            v0,
            limits: l,
            ..
        } => {
            v.p(rate, Kind::Frequency, "controller.rate", Pos);
            v.p(v0, Kind::Voltage, "controller.v0", NonNeg);
            limits(&mut v, l);
        }
        ControllerParams::Foc {
            current,
            velocity,
            position,
            limits: l,
            ..
        } => {
            loop_(&mut v, current, "controller.current");
            for (lp, at) in [
                (velocity, "controller.velocity"),
                (position, "controller.position"),
            ] {
                if let Some(lp) = lp {
                    loop_(&mut v, lp, at);
                }
            }
            limits(&mut v, l);
        }
        ControllerParams::Mit {
            current,
            rate,
            tau_ff,
            limits: l,
            ..
        } => {
            loop_(&mut v, current, "controller.current");
            v.p(rate, Kind::Frequency, "controller.rate", Pos);
            v.o(tau_ff, Kind::Torque, "controller.tau_ff", Any);
            limits(&mut v, l);
        }
        ControllerParams::Custom {
            rate, limits: l, ..
        } => {
            v.p(rate, Kind::Frequency, "controller.rate", Pos);
            limits(&mut v, l);
        }
    }
    v.out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::parse_yaml;

    fn rejects(issues: &[Issue]) -> Vec<String> {
        issues
            .iter()
            .filter(|i| i.severity == Severity::Reject)
            .map(|i| i.path.clone())
            .collect()
    }

    #[test]
    fn bad_components_are_rejected() {
        let inv: InverterParams = parse_yaml(
            "name: x\nf_pwm: 20 mH\ndead_time: 1 us\nr_on: 1 mohm\nv_f: 0.7 V\nt_rise: 1 ns\nt_fall: 1 ns\n",
            "i",
        )
        .unwrap();
        assert_eq!(rejects(&validate_inverter(&inv)), ["inverter.f_pwm"]);
        let g: GearboxParams = parse_yaml("name: g\nratio: -3\nefficiency: 7\n", "g").unwrap();
        assert_eq!(
            rejects(&validate_gearbox(&g)),
            ["gearbox.ratio", "gearbox.efficiency"]
        );
        let b: SupplyParams = parse_yaml(
            "kind: battery\nchemistry: lipo\nseries: 0\nparallel: 1\ncapacity_cell: 2 Ah\nr0_cell: 1 mohm\nr1_cell: 1 mohm\nc1_cell: 100 F\nsoc_init: 4\nocv_cell: [[0.5, 3.7], [0.5, 3.8]]\n",
            "b",
        )
        .unwrap();
        assert_eq!(
            rejects(&validate_supply(&b)),
            ["supply.series", "supply.soc_init", "supply.ocv_cell"]
        );
        let l: LoadParams = parse_yaml("kind: flywheel\ninertia: 0 kg*m^2\n", "l").unwrap();
        assert_eq!(rejects(&validate_load(&l)), ["load.inertia"]);
        let s: SensorsParams =
            parse_yaml("adc: { bits: 12, full_scale: 20 A, shunts: 4 }\n", "s").unwrap();
        assert_eq!(rejects(&validate_sensors(&s)), ["sensors.adc.shunts"]);
        let c: ControllerParams = parse_yaml(
            "family: foc\ncurrent: { rate: 20 kHz, bandwidth: 9 kHz }\nlimits: { current: -1 A }\n",
            "c",
        )
        .unwrap();
        let issues = validate_controller(&c);
        assert_eq!(rejects(&issues), ["controller.limits.current"]);
        assert!(
            issues
                .iter()
                .any(|i| i.severity == Severity::Warn && i.path == "controller.current.bandwidth")
        );
    }
}

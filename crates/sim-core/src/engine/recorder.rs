//! Recorder (P03.T10): streaming min/max decimation (plots show ripple and spikes
//! even when decimated) and full-rate capture for export, with a memory cap.

use std::collections::VecDeque;

use serde::Serialize;

use super::signals::{SignalBus, SignalId};

/// One decimated bucket per signal: (min, max, last).
#[derive(Debug, Clone, Serialize)]
pub struct StreamFrame {
    /// Start time of each bucket [s].
    pub t: Vec<f64>,
    /// Per subscribed signal: per bucket min / max.
    pub min: Vec<Vec<f64>>,
    pub max: Vec<Vec<f64>>,
}

#[derive(Debug, Clone)]
pub struct Recorder {
    ids: Vec<SignalId>,
    bucket: f64,
    bucket_start: f64,
    cur_min: Vec<f64>,
    cur_max: Vec<f64>,
    open: bool,
    stream: VecDeque<(f64, Vec<f64>, Vec<f64>)>,
    stream_cap: usize,
    capture: Option<Capture>,
}

/// Full-rate capture buffer.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Capture {
    pub t: Vec<f64>,
    /// Row-major flat buffer: `n_signals` values per time sample (no per-sample allocation).
    pub data: Vec<f64>,
    pub n_signals: usize,
    pub max_rows: usize,
    /// Set when the cap was reached (later samples dropped).
    pub truncated: bool,
}

impl Capture {
    /// Row `k` (values of all captured signals at `t[k]`).
    pub fn row(&self, k: usize) -> &[f64] {
        &self.data[k * self.n_signals..(k + 1) * self.n_signals]
    }
}

impl Recorder {
    /// `bucket` = decimation interval [s] (e.g. 1/200 for 200 buckets/s).
    pub fn new(ids: Vec<SignalId>, bucket: f64, stream_cap: usize) -> Self {
        let n = ids.len();
        Self {
            ids,
            bucket,
            bucket_start: 0.0,
            cur_min: vec![f64::INFINITY; n],
            cur_max: vec![f64::NEG_INFINITY; n],
            open: false,
            stream: VecDeque::new(),
            stream_cap,
            capture: None,
        }
    }

    pub fn ids(&self) -> &[SignalId] {
        &self.ids
    }

    /// Start full-rate capture of the subscribed signals (≤ `max_rows` samples).
    pub fn start_capture(&mut self, max_rows: usize) {
        let n = self.ids.len();
        self.capture = Some(Capture {
            max_rows,
            n_signals: n,
            t: Vec::with_capacity(max_rows.min(1 << 20)),
            data: Vec::with_capacity((max_rows * n).min(1 << 22)),
            truncated: false,
        });
    }

    pub fn stop_capture(&mut self) -> Option<Capture> {
        self.capture.take()
    }

    /// Record the bus at time `t` (called after each accepted substep / event).
    pub fn sample(&mut self, t: f64, bus: &SignalBus) {
        if !self.open {
            self.bucket_start = (t / self.bucket).floor() * self.bucket;
            self.open = true;
        }
        while t >= self.bucket_start + self.bucket {
            self.close_bucket();
            self.bucket_start += self.bucket;
        }
        for (k, &id) in self.ids.iter().enumerate() {
            let v = bus.get(id);
            self.cur_min[k] = self.cur_min[k].min(v);
            self.cur_max[k] = self.cur_max[k].max(v);
        }
        if let Some(c) = &mut self.capture {
            if c.t.len() < c.max_rows {
                c.t.push(t);
                c.data.extend(self.ids.iter().map(|&id| bus.get(id)));
            } else {
                c.truncated = true;
            }
        }
    }

    fn close_bucket(&mut self) {
        if self.cur_min.iter().all(|v| v.is_infinite()) {
            return; // empty bucket (no samples): skip
        }
        if self.stream.len() == self.stream_cap {
            self.stream.pop_front(); // drop oldest; never block the engine
        }
        let n = self.ids.len();
        let min = std::mem::replace(&mut self.cur_min, vec![f64::INFINITY; n]);
        let max = std::mem::replace(&mut self.cur_max, vec![f64::NEG_INFINITY; n]);
        self.stream.push_back((self.bucket_start, min, max));
    }

    /// Completed buckets since the last drain, column-oriented per signal.
    pub fn drain_stream(&mut self) -> StreamFrame {
        let n = self.ids.len();
        let mut f = StreamFrame {
            t: Vec::new(),
            min: vec![Vec::new(); n],
            max: vec![Vec::new(); n],
        };
        for (t, mn, mx) in self.stream.drain(..) {
            f.t.push(t);
            for k in 0..n {
                f.min[k].push(mn[k]);
                f.max[k].push(mx[k]);
            }
        }
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::signals::SignalKind;

    #[test]
    fn decimation_keeps_ripple_envelope() {
        let mut bus = SignalBus::new();
        let id = bus
            .register("test.square", "-", "", SignalKind::Diagnostic)
            .unwrap();
        let mut rec = Recorder::new(vec![id], 0.01, 1000); // 100 buckets/s
        // 20 kHz square wave sampled at 400 kHz for 0.1 s.
        let dt = 2.5e-6;
        for k in 0..40_000 {
            let t = k as f64 * dt;
            let v = if ((t * 20_000.0 * 2.0).floor() as i64) % 2 == 0 {
                1.0
            } else {
                -1.0
            };
            bus.set(id, v);
            rec.sample(t, &bus);
        }
        let f = rec.drain_stream();
        assert_eq!(f.t.len(), 9, "10 buckets, last still open");
        assert!(f.min[0].iter().all(|&m| m == -1.0) && f.max[0].iter().all(|&m| m == 1.0));
    }

    #[test]
    fn capture_respects_cap() {
        let mut bus = SignalBus::new();
        let id = bus
            .register("test.x", "-", "", SignalKind::Diagnostic)
            .unwrap();
        let mut rec = Recorder::new(vec![id], 1.0, 10);
        rec.start_capture(5);
        for k in 0..8 {
            bus.set(id, k as f64);
            rec.sample(k as f64 * 0.1, &bus);
        }
        let c = rec.stop_capture().unwrap();
        assert_eq!(c.t.len(), 5);
        assert!(c.truncated);
        assert_eq!(c.row(4), &[4.0]);
    }
}

//! Sensors (EQ-SENS-*): the common non-ideality pipeline and the sensor blocks.

pub mod adc;
pub mod encoder_incremental;
pub mod encoder_magnetic;
pub mod hall;
pub mod pipeline;

use crate::engine::block::DiscreteBlock;
use crate::engine::rng::RngService;
use crate::engine::signals::{SignalBus, SignalError};
use crate::engine::time::SimTime;

use adc::{Adc, AdcConfig};
use encoder_incremental::{IncrementalConfig, IncrementalEncoder};
use encoder_magnetic::{MagneticEncoder, MagneticEncoderConfig};
use hall::{HallBlock, HallConfig};
use pipeline::{PipelineConfig, SensorPipeline};

/// The angle encoder of a scene.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EncoderSpec {
    Magnetic(MagneticEncoderConfig),
    Incremental(IncrementalConfig, PipelineConfig),
}

/// A sampled sensor: its model and sampling (period, first-sample phase).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sampled<T> {
    pub cfg: T,
    pub period: SimTime,
    pub phase: SimTime,
}

/// The sensors a scene builds (all optional), as plain data. `blocks` turns them into
/// discrete blocks once the plant has registered `motor.*` (and `bus.v`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SensorSpec {
    /// Scene seed: each block draws from its own substream (CONVENTIONS §6).
    pub seed: u64,
    pub hall: Option<Sampled<(HallConfig, PipelineConfig)>>,
    pub encoder: Option<Sampled<EncoderSpec>>,
    pub adc: Option<Sampled<AdcConfig>>,
}

impl SensorSpec {
    pub fn blocks(&self, bus: &mut SignalBus) -> Result<Vec<Box<dyn DiscreteBlock>>, SignalError> {
        let rng = RngService::new(self.seed);
        let mut out: Vec<Box<dyn DiscreteBlock>> = vec![];
        if let Some(h) = &self.hall {
            let pipe = SensorPipeline::new(h.cfg.1, rng.stream("sensors.hall"));
            out.push(Box::new(
                HallBlock::new(h.cfg.0, h.period, pipe, bus)?.with_phase(h.phase),
            ));
        }
        if let Some(e) = &self.encoder {
            let rng = rng.stream("sensors.encoder");
            out.push(match &e.cfg {
                EncoderSpec::Magnetic(c) => {
                    Box::new(MagneticEncoder::new(c, e.period, rng, bus)?.with_phase(e.phase))
                }
                EncoderSpec::Incremental(c, p) => {
                    let pipe = SensorPipeline::new(*p, rng);
                    Box::new(IncrementalEncoder::new(*c, e.period, pipe, bus)?.with_phase(e.phase))
                }
            });
        }
        if let Some(a) = &self.adc {
            out.push(Box::new(
                Adc::new(&a.cfg, a.period, &rng, bus)?.with_phase(a.phase),
            ));
        }
        Ok(out)
    }
}

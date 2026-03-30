use core::f64;

use vexide::{math::Angle, prelude::{AdiEncoder, Motor, RotationSensor}, smart::PortError};

pub(crate) enum EncoderType {
    AdiEncoder,
    Motor,
    RotationSensor,
}

pub(crate) trait Encoder {
    type Error;

    fn position(&self) -> Result<Angle, Self::Error>;
    fn encoder_type(&self) -> EncoderType;
    fn reset(&mut self) -> Result<(), Self::Error>;
}

impl Encoder for RotationSensor {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
    }

    fn encoder_type(&self) -> EncoderType {
        EncoderType::RotationSensor
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        self.reset_position()
    }
}

impl Encoder for Motor {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
    }

    fn encoder_type(&self) -> EncoderType {
        EncoderType::Motor
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        self.reset_position()
    }
}

impl<const TICKS: u32> Encoder for AdiEncoder<TICKS> {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
    }

    fn encoder_type(&self) -> EncoderType {
        EncoderType::AdiEncoder
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        self.reset_position()
    }
}

impl<T: Encoder<Error = PortError>> Encoder for Vec<T> {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        let mut n_encoders: usize = 0;
        let mut total_val = Angle::ZERO;
        let mut last_err = None;

        for encoder in 0..self.len() {
            let val = self[encoder].position();
            match val {
                Ok(v) => {
                    total_val += v;
                    n_encoders += 1;
                }
                Err(e) => {
                    last_err = Some(e);
                }
            };
        };

        if n_encoders == 0 && let Some(err) = last_err {
            return Err(err);
        }

        total_val /= n_encoders as f64;
        Ok(total_val)
    }

    fn encoder_type(&self) -> EncoderType {
        if self.is_empty() { EncoderType::Motor } else { self.first().unwrap().encoder_type() }
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        let mut last_err: Option<Self::Error> = None;
        if self.is_empty() { return Ok(()) }
        for i in 0..(self.len()-1) {
            last_err = match self[i].reset() {
                Ok(_) => last_err,
                Err(e) => Some(e),
            };
        };
        if let Some(err) = last_err {
            Err(err)
        } else {
            Ok(())
        }
    }
}

pub(crate) struct TrackingWheel {
    encoder: Box<dyn Encoder<Error = PortError>>,
    diameter: f64,
    offset: f64,
    gear_ratio: f64,
}

impl TrackingWheel {
    pub(crate) fn new(encoder: Box<dyn Encoder<Error = PortError>>, diameter: f64, offset: f64, gear_ratio: f64) -> Self {
        Self {
            encoder, diameter, offset, gear_ratio
        }
    }

    pub(crate) fn reset(&mut self) -> Result<(), PortError> {
        self.encoder.reset()
    }

    pub(crate) fn get_distance_traveled(&self) -> Result<f64, PortError> {
        Ok(self.encoder.position()?.as_radians() * self.diameter * f64::consts::PI / self.gear_ratio)
    }

    pub(crate) fn get_offset(&self) -> f64 {
        self.offset
    }

    pub(crate) fn get_type(&self) -> EncoderType {
        self.encoder.encoder_type()
    }
}
use vexide::{math::Angle, prelude::{AdiEncoder, Motor, RotationSensor}, smart::PortError};

trait Encoder {
    type Error;

    fn position(&self) -> Result<Angle, Self::Error>;
}

impl Encoder for RotationSensor {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
    }
}

impl Encoder for Motor {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
    }
}

impl<const TICKS: u32> Encoder for AdiEncoder<TICKS> {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
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

        if n_encoders == 0 && last_err != None {
            return Err(last_err.unwrap());
        }

        total_val /= n_encoders as f64;
        Ok(total_val)
    }
}

pub(super) struct TrackingWheel {
    encoder: Box<dyn Encoder<Error = PortError>>,
    diameter: f64,
}
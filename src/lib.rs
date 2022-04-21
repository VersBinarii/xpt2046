#![doc(html_root_url = "https://docs.rs/xpt2046")]
#![doc(issue_tracker_base_url = "https://github.com/VersBinarii/xpt2046/issues/")]
#![deny(
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    unused_variables,
    unreachable_code,
    unused_comparisons,
    unused_must_use
)]
#![no_std]

//! A platform agnostic Rust driver for XPT2046 touch controller, based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!

use crate::calibration::{calculate_calibration, calibration_draw_point};
pub use crate::{
    calibration::CalibrationPoint,
    error::{BusError, Error},
    exti_pin::Xpt2046Exti,
};
use core::{fmt::Debug, ops::RemAssign};
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::{Rgb565, RgbColor},
};
use embedded_hal::{
    delay::blocking::DelayUs, digital::blocking::OutputPin, spi::blocking::Transfer,
};

#[cfg(feature = "with_defmt")]
use defmt::{write, Format, Formatter};

pub mod calibration;
pub mod error;
pub mod exti_pin;

const CHANNEL_SETTING_X: u8 = 0b10010000;
const CHANNEL_SETTING_Y: u8 = 0b11010000;

const MAX_SAMPLES: usize = 128;
const TX_BUFF_LEN: usize = 5;

#[derive(Debug)]
pub struct CalibrationData {
    pub alpha_x: f32,
    pub beta_x: f32,
    pub delta_x: f32,
    pub alpha_y: f32,
    pub beta_y: f32,
    pub delta_y: f32,
}

/// Orientation of the touch screen
#[derive(Debug)]
pub enum Orientation {
    Portrait,
    PortraitFlipped,
    Landscape,
    LandscapeFlipped,
}

impl Orientation {
    /// Default location for the test touch point
    /// Those depend on whether the touch screen operates in
    /// Portrait or Landscape position
    pub fn calibration_point(&self) -> CalibrationPoint {
        match self {
            Orientation::Portrait | Orientation::PortraitFlipped => CalibrationPoint {
                a: Point::new(10, 10),
                b: Point::new(80, 210),
                c: Point::new(200, 170),
            },
            Orientation::Landscape | Orientation::LandscapeFlipped => CalibrationPoint {
                a: Point::new(20, 25),
                b: Point::new(160, 220),
                c: Point::new(300, 110),
            },
        }
    }

    /// Default calibration values used for calculating the touch points
    /// Those depend on whether the touch screen operates in
    /// Portrait or Landscape position
    pub fn calibration_data(&self) -> CalibrationData {
        match self {
            Orientation::Portrait => CalibrationData {
                alpha_x: -0.0009337,
                beta_x: -0.0636839,
                delta_x: 250.342,
                alpha_y: -0.0889775,
                beta_y: -0.00118110,
                delta_y: 356.538,
            },
            Orientation::PortraitFlipped => CalibrationData {
                alpha_x: 0.0006100,
                beta_x: 0.0647828,
                delta_x: -13.634,
                alpha_y: 0.0890609,
                beta_y: 0.0001381,
                delta_y: -35.73,
            },
            Orientation::Landscape => CalibrationData {
                alpha_x: -0.0885542,
                beta_x: 0.0016532,
                delta_x: 349.800,
                alpha_y: 0.0007309,
                beta_y: 0.06543699,
                delta_y: -15.290,
            },
            Orientation::LandscapeFlipped => CalibrationData {
                alpha_x: 0.0902216,
                beta_x: 0.0006510,
                delta_x: -38.657,
                alpha_y: -0.0010005,
                beta_y: -0.0667030,
                delta_y: 258.08,
            },
        }
    }
}

/// Current state of the driver
#[derive(PartialEq, Debug)]
pub enum TouchScreenState {
    /// Driver waith for touch
    IDLE,
    /// Driver debounces the touch
    PRESAMPLING,
    /// Confirmed touch
    TOUCHED,
    /// Touch released
    RELEASED,
}

#[derive(Debug, PartialEq)]
pub enum TouchScreenOperationMode {
    /// Normal touch reading
    NORMAL,
    /// Manual calibration mode
    CALIBRATION,
}

#[derive(Debug)]
pub struct TouchSamples {
    /// All the touch samples
    samples: [Point; MAX_SAMPLES],
    /// current number of captured samples
    counter: usize,
}

impl Default for TouchSamples {
    fn default() -> Self {
        Self {
            counter: 0,
            samples: [Point::default(); MAX_SAMPLES],
        }
    }
}

impl TouchSamples {
    pub fn average(&self) -> Point {
        let mut x = 0;
        let mut y = 0;

        for point in self.samples {
            x += point.x;
            y += point.y;
        }
        x /= MAX_SAMPLES as i32;
        y /= MAX_SAMPLES as i32;
        Point::new(x, y)
    }
}

#[derive(Debug)]
pub struct Xpt2046<SPI, CS, PinIRQ> {
    /// THe SPI interface
    spi: SPI,
    /// Control pin
    cs: CS,
    /// Interrupt control pin
    irq: PinIRQ,
    /// Internall buffers tx
    tx_buff: [u8; TX_BUFF_LEN],
    /// Internal buffer for rx
    rx_buff: [u8; TX_BUFF_LEN],
    /// Current driver state
    screen_state: TouchScreenState,
    /// Buffer for the touch data samples
    ts: TouchSamples,
    calibration_data: CalibrationData,
    operation_mode: TouchScreenOperationMode,
    /// Location of the touch points used for
    /// performing manual calibration
    calibration_point: CalibrationPoint,
}

impl<SPI, CS, PinIRQ> Xpt2046<SPI, CS, PinIRQ>
where
    SPI: Transfer<u8>,
    CS: OutputPin,
    PinIRQ: Xpt2046Exti,
{
    pub fn new(spi: SPI, cs: CS, irq: PinIRQ, orientation: Orientation) -> Self {
        Self {
            spi,
            cs,
            irq,
            tx_buff: [0; TX_BUFF_LEN],
            rx_buff: [0; TX_BUFF_LEN],
            screen_state: TouchScreenState::IDLE,
            ts: TouchSamples::default(),
            calibration_data: orientation.calibration_data(),
            operation_mode: TouchScreenOperationMode::NORMAL,
            calibration_point: orientation.calibration_point(),
        }
    }
}

impl<SPI, CS, PinIRQ, SPIError, CSError> Xpt2046<SPI, CS, PinIRQ>
where
    SPI: Transfer<u8, Error = SPIError>,
    CS: OutputPin<Error = CSError>,
    PinIRQ: Xpt2046Exti,
    SPIError: Debug,
    CSError: Debug,
{
    fn spi_read(&mut self) -> Result<(), Error<BusError<SPIError, CSError>>> {
        self.cs
            .set_low()
            .map_err(|e| Error::Bus(BusError::Pin(e)))?;
        self.spi
            .transfer(&mut self.rx_buff, &self.tx_buff)
            .map_err(|e| Error::Bus(BusError::Spi(e)))?;
        self.cs
            .set_high()
            .map_err(|e| Error::Bus(BusError::Pin(e)))?;
        Ok(())
    }

    /// Read raw values from the XPT2046 driver
    fn read_xy(&mut self) -> Result<Point, Error<BusError<SPIError, CSError>>> {
        self.spi_read()?;

        let x = (self.rx_buff[1] as i32) << 8 | self.rx_buff[2] as i32;
        let y = (self.rx_buff[3] as i32) << 8 | self.rx_buff[4] as i32;
        Ok(Point::new(x, y))
    }

    /// Read the calibrated point of touch from XPT2046
    fn read_touch_point(&mut self) -> Result<Point, Error<BusError<SPIError, CSError>>> {
        let raw_point = self.read_xy()?;

        let (x, y) = match self.operation_mode {
            TouchScreenOperationMode::NORMAL => {
                let x = self.calibration_data.alpha_x * raw_point.x as f32
                    + self.calibration_data.beta_x * raw_point.y as f32
                    + self.calibration_data.delta_x;
                let y = self.calibration_data.alpha_y * raw_point.x as f32
                    + self.calibration_data.beta_y * raw_point.y as f32
                    + self.calibration_data.delta_y;
                (x as i32, y as i32)
            }
            TouchScreenOperationMode::CALIBRATION => {
                /*
                 * We're running calibration so just return raw
                 * point measurements without compensation
                 */
                (raw_point.x, raw_point.y)
            }
        };
        Ok(Point::new(x, y))
    }

    pub fn get_touch_point(&self) -> Point {
        self.ts.average()
    }

    /// Check if the display is currently touched
    pub fn is_touched(&self) -> bool {
        self.screen_state == TouchScreenState::TOUCHED
    }

    /// Sometimes the TOUCHED state needs to be cleared
    pub fn clear_touch(&mut self) {
        self.screen_state = TouchScreenState::PRESAMPLING;
    }

    /// Reset the driver and preload tx buffer with register data
    pub fn init<D: DelayUs>(&mut self, delay: &mut D) {
        self.tx_buff[0] = 0x80;
        self.cs.set_high().unwrap();
        self.spi_read().unwrap();
        delay.delay_ms(1).unwrap();

        /*
         * Load the tx_buffer with the channels config
         * for all subsequent reads
         * The byte shifting provides padding to align the read bytes with the
         * DCLK. XPT2046 datasheet figure 12
         */
        self.tx_buff = [
            CHANNEL_SETTING_X >> 3,
            CHANNEL_SETTING_X << 5,
            CHANNEL_SETTING_Y >> 3,
            CHANNEL_SETTING_Y << 5,
            0,
        ];
    }

    /// continually runs and and collects the touch data from xpt2046
    pub fn run(&mut self, exti: &mut PinIRQ::Exti) {
        match self.screen_state {
            TouchScreenState::IDLE => {
                if self.operation_mode == TouchScreenOperationMode::CALIBRATION && self.irq.is_low()
                {
                    self.screen_state = TouchScreenState::PRESAMPLING;
                }
            }
            TouchScreenState::PRESAMPLING => {
                if self.irq.is_high() {
                    self.screen_state = TouchScreenState::RELEASED
                }
                let point_sample = self.read_touch_point().unwrap();
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                if self.ts.counter + 1 == MAX_SAMPLES {
                    self.ts.counter = 0;
                    self.screen_state = TouchScreenState::TOUCHED;
                }
            }
            TouchScreenState::TOUCHED => {
                let point_sample = self.read_touch_point().unwrap();
                self.ts.samples[self.ts.counter] = point_sample;
                self.ts.counter += 1;
                /*
                 * Wrap around the counter if the screen
                 * is touched for longer time
                 */
                self.ts.counter.rem_assign(MAX_SAMPLES - 1);
                if self.irq.is_high() {
                    self.screen_state = TouchScreenState::RELEASED
                }
            }
            TouchScreenState::RELEASED => {
                self.screen_state = TouchScreenState::IDLE;
                self.ts.counter = 0;
                /*
                 * The PENIRQ should be re-enabled in here
                 * as we finished sending any data to the touch controller
                 */
                self.irq.clear_interrupt();
                self.irq.enable_interrupt(exti);
            }
        }
    }

    /// This function should be only ever be called in an EXTI
    /// interrupt handler.
    pub fn exti_irq_handle(&mut self, exti: &mut PinIRQ::Exti) {
        /*
         * Disable the PENIRQ so that it wont be false triggering our handler
         * as per XPT2046 Touch Screen Controller datasheet page 25
         *
         * It is recommended that the processor mask the interrupt PENIRQ
         * is associated with whenever the processor sends
         * a control byte to the XPT2046. This prevents false triggering of
         * interrupts when the PENIRQ output is disabled in the cases
         * discussed in this section.
         */
        self.irq.disable_interrupt(exti);
        self.irq.clear_interrupt();
        self.screen_state = TouchScreenState::PRESAMPLING;
    }

    /// Collects the reading for 3 sample points and
    /// calculates a set of calibration data. The default calibration data seem
    /// to work ok but if for some reason touch screen needs to be recalibrated
    /// then look no further.
    /// This should be run after init() method.
    pub fn calibrate<DT, DELAY>(&mut self, dt: &mut DT, delay: &mut DELAY, exti: &mut PinIRQ::Exti)
    where
        DT: DrawTarget<Color = Rgb565>,
        DELAY: DelayUs,
    {
        let mut calibration_count = 0;
        let mut new_a = Point::zero();
        let mut new_b = Point::zero();
        let mut new_c = Point::zero();
        let old_cp = self.calibration_point.clone();
        // Prepare the screen for points
        let _ = dt.clear(Rgb565::BLACK);

        // Set correct state to fetch raw data from touch controller
        self.operation_mode = TouchScreenOperationMode::CALIBRATION;
        while calibration_count < 4 {
            // We must run our state machine to capture user input
            self.run(exti);
            match calibration_count {
                0 => {
                    calibration_draw_point(dt, &old_cp.a);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_a = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                1 => {
                    calibration_draw_point(dt, &old_cp.b);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_b = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }
                2 => {
                    calibration_draw_point(dt, &old_cp.c);
                    if self.screen_state == TouchScreenState::TOUCHED {
                        new_c = self.get_touch_point();
                    }
                    if self.screen_state == TouchScreenState::RELEASED {
                        let _ = delay.delay_ms(200);
                        calibration_count += 1;
                    }
                }

                3 => {
                    // Create new calibration point from the captured samples
                    self.calibration_point = CalibrationPoint {
                        a: new_a,
                        b: new_b,
                        c: new_c,
                    };
                    // and then re-caculate calibration
                    self.calibration_data = calculate_calibration(&old_cp, &self.calibration_point);
                    calibration_count += 1;
                }
                _ => {}
            }
        }

        let _ = dt.clear(Rgb565::WHITE);
        self.operation_mode = TouchScreenOperationMode::NORMAL;
    }
}

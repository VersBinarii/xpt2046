use super::CalibrationData;
use embedded_graphics::{
    primitives::{Line, Primitive, PrimitiveStyle},
    Drawable,
};
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::{Rgb565, RgbColor},
};

#[cfg(feature = "with_defmt")]
use defmt::{write, Format, Formatter};

#[derive(Debug, Clone)]
pub struct CalibrationPoint {
    pub a: Point,
    pub b: Point,
    pub c: Point,
}

impl CalibrationPoint {
    pub fn delta(&self) -> i32 {
        (self.a[0] - self.c[0]) * (self.b[1] - self.c[1])
            - (self.b[0] - self.c[0]) * (self.a[1] - self.c[1])
    }
}

#[cfg(feature = "with_defmt")]
impl Format for CalibrationPoint {
    fn format(&self, fmt: Formatter) {
        write!(
            fmt,
            "a[x: {} y:{}]\nb[x: {} y: {}]\nc[x:{} y:{}]",
            self.a.x, self.a.y, self.b.x, self.b.y, self.c.x, self.c.y,
        )
    }
}
pub(crate) fn calibration_draw_point<DT: DrawTarget<Color = Rgb565>>(dt: &mut DT, p: &Point) {
    let _ = Line::new(Point::new(p.x - 4, p.y), Point::new(p.x + 4, p.y))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
    let _ = Line::new(Point::new(p.x, p.y - 4), Point::new(p.x, p.y + 4))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(dt);
}

pub(crate) fn calculate_calibration(
    old_cp: &CalibrationPoint,
    new_cp: &CalibrationPoint,
) -> CalibrationData {
    let delta = new_cp.delta() as f32;

    let alpha_x = ((old_cp.a[0] - old_cp.c[0]) * (new_cp.b[1] - new_cp.c[1])
        - (old_cp.b[0] - old_cp.c[0]) * (new_cp.a[1] - new_cp.c[1])) as f32
        / delta;

    let beta_x = ((new_cp.a[0] - new_cp.c[0]) * (old_cp.b[0] - old_cp.c[0])
        - (new_cp.b[0] - new_cp.c[0]) * (old_cp.a[0] - old_cp.c[0])) as f32
        / delta;

    let delta_x = ((old_cp.a[0]) * (new_cp.b[0] * new_cp.c[1] - new_cp.c[0] * new_cp.b[1])
        - (old_cp.b[0]) * (new_cp.a[0] * new_cp.c[1] - new_cp.c[0] * new_cp.a[1])
        + (old_cp.c[0]) * (new_cp.a[0] * new_cp.b[1] - new_cp.b[0] * new_cp.a[1]))
        as f32
        / delta;

    let alpha_y = ((old_cp.a[1] - old_cp.c[1]) * (new_cp.b[1] - new_cp.c[1])
        - (old_cp.b[1] - old_cp.c[1]) * (new_cp.a[1] - new_cp.c[1])) as f32
        / delta;

    let beta_y = ((new_cp.a[0] - new_cp.c[0]) * (old_cp.b[1] - old_cp.c[1])
        - (new_cp.b[0] - new_cp.c[0]) * (old_cp.a[1] - old_cp.c[1])) as f32
        / delta;

    let delta_y = ((old_cp.a[1]) * (new_cp.b[0] * new_cp.c[1] - new_cp.c[0] * new_cp.b[1])
        - (old_cp.b[1]) * (new_cp.a[0] * new_cp.c[1] - new_cp.c[0] * new_cp.a[1])
        + (old_cp.c[1]) * (new_cp.a[0] * new_cp.b[1] - new_cp.b[0] * new_cp.a[1]))
        as f32
        / delta;

    CalibrationData {
        alpha_x,
        beta_x,
        delta_x,
        alpha_y,
        beta_y,
        delta_y,
    }
}

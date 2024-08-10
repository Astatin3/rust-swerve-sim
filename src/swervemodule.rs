use std::process::id;
use piston_window::{color, Context, Ellipse, G2d, Line, line, rectangle};
use piston_window::math::Scalar;
use crate::swerve_math;
use crate::swerve_math::{get_heading, get_heading_x, get_heading_y, Vector2d};

#[derive(Clone, Copy)]
pub struct SwerveModule {
    pub(crate) id:i32,
    pub(crate) swerve_rot: f32,
    pub(crate) vec: Vector2d,

    // pub(crate) vec_x: f32,
    // pub(crate) vec_y: f32,
    // pub(crate) vec_speed: f32,
    // pub(crate) vec_angle: f32,
}
impl SwerveModule {
    pub const fn new(id: i32) -> Self {
        SwerveModule {
            id:id,
            swerve_rot:0.,
            vec:Vector2d {x:0.,y:0.}
        }
    }
    pub fn draw(&mut self, c: &Context, mut g: &mut G2d, screen_x:u32, screen_y:u32) {
    // pub fn draw(c: &Context, mut g: &mut G2d) {
        Ellipse::new_border(color::WHITE, 1.0).draw(
            rectangle::centered_square(screen_x as Scalar, screen_y as Scalar, 50.0),
            &c.draw_state,
            c.transform,
            g,
        );

        // let vec_x = get_heading_x(self.vec_angle) * 50.;
        // let vec_y = get_heading_y(self.vec_angle * 50.);

        line(color::WHITE, 1.0,
             [screen_x as f64,
                 screen_y  as f64,
                 (screen_x as f32 + self.vec.x*500.) as f64,
                 (screen_y as f32 + self.vec.y*500.) as f64],
             c.transform, g);
    }

    pub fn get_rotation(&mut self) -> f32 {
        if self.id == 1 {
            return ((-45. + self.vec.get_ang() - self.swerve_rot) % 360.);
        }else if self.id == 2 {
            return ((45. + self.vec.get_ang() - self.swerve_rot) % 360.);
        }else if self.id == 3 {
            return ((135. + self.vec.get_ang() - self.swerve_rot) % 360.);
        }else if self.id == 4 {
            return ((-135. + self.vec.get_ang() - self.swerve_rot) % 360.);
        }
        return 0.;
    }

    fn get_swerve_corner_x(&mut self, id: i8) -> f32 {
        if id == 1 {
            return swerve_math::get_heading_x(-45. + self.swerve_rot);
        }else if id == 2 {
            return swerve_math::get_heading_x(45. + self.swerve_rot);
        }else if id == 3 {
            return swerve_math::get_heading_x(135. + self.swerve_rot);
        }else if id == 4 {
            return swerve_math::get_heading_x(-135. + self.swerve_rot);
        }
        return 0.;
    }

    fn get_swerve_corner_y(&mut self, id: i8) -> f32 {
        if id == 1 {
            return swerve_math::get_heading_y(45. + self.swerve_rot);
        }else if id == 2 {
            return swerve_math::get_heading_y(45. + self.swerve_rot);
        }else if id == 3 {
            return swerve_math::get_heading_y(135. + self.swerve_rot);
        }else if id == 4 {
            return swerve_math::get_heading_y(-135. + self.swerve_rot);
        }
        return 0.;
    }
}
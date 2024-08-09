use piston_window::{color, Context, Ellipse, G2d, rectangle};
use crate::swerve_math;

pub struct SwerveModule {
    id:i32,
    w:i32,
    h: i32,
    swerve_rot: f32
}
impl SwerveModule {

    /**
     * Draws the wheel vector given wheel data and screen data.
     * @param g the Graphics2D object to draw with
     * @param angle the angle of the wheel, in degrees
     * @param speed the speed of the wheel [-1, 1]
     * @param font the font size currently in use
     * @param startFactor the factor [0, 1] to start the vector at, when scaling towards the center
     * @param width the width of the swerve box (in pixels)
     * @param height the height of the swerve box (in pixels)
     * @param vecScale the scalar for the wheel vector
     */
    pub fn draw(c: &Context, mut g: &mut G2d,
        angle:f32, speed:f32, font:i32, startFactor:f32, width:f32, height:f32, vecScale:f32) {
    // pub fn draw(c: &Context, mut g: &mut G2d) {
        Ellipse::new_border(color::WHITE, 1.0).draw(
            rectangle::centered_square(10.0, 10.0, 50.0),
            &c.draw_state,
            c.transform,
            g,
        );
    }

    fn get_swerve_corner_x(id: i8) -> f32 {
        if (id == 1) {
            return swerve_math::get_heading_x(-45 + swerve_rot);
        }else if (id == 2) {
            return swerve_math::get_heading_x(45 + swerve_rot);
        }else if (id == 3) {
            return swerve_math::get_heading_x(135 + swerve_rot);
        }else if (id == 4) {
            return swerve_math::get_heading_x(-135 + swerve_rot);
        }
        return 0;
    }
}
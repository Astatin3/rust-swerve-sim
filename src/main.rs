mod swerve_math;
mod swervemodule;
mod pid;

extern crate piston_window;

use std::thread;
use std::time::Duration;
use piston_window::*;
use piston_window::math::Scalar;
use piston_window::rectangle::square;
use piston_window::types::Color;
use crate::swerve_math::{degrees_to_radians, get_heading, get_heading_x, get_heading_y, Vector2d};
use crate::swervemodule::SwerveModule;


static SCREEN_SQUARE_SIZE:u32 = 1024;
static SWERVE_CENTER_DISTANCE:u32 = SCREEN_SQUARE_SIZE/8;

static XY_ACCEL:f32 = 0.2;
static ROT_ACCEL:f32 = 0.001;
static MAX_ROT_VEL:f32 = 32.;

static VECTOR_SCALE:f32 = 20.;

static XY_FRICTION:f32 = 0.95;
static ROT_FRICTION:f32 = 0.998;
// static mut TIME: u64 = 0;


struct but {
    key_w: bool,
    key_a: bool,
    key_s: bool,
    key_d: bool,

    key_q: bool,
    key_e: bool,
}

static mut BUTTONS: but = but {
    key_w: false, key_a: false, key_s: false, key_d: false, key_q: false, key_e: false,
};

static mut VEL_X: f32 = 0.;
static mut VEL_Y: f32 = 0.;
static mut VEL_R: f32 = 0.;

static mut POS_X: f32 = 0.;
static mut POS_Y: f32 = 0.;
static mut POS_R: f32 = 0.;

static mut W1:SwerveModule = SwerveModule::new(1);
static mut W2:SwerveModule = SwerveModule::new(2);
static mut W3:SwerveModule = SwerveModule::new(3);
static mut W4:SwerveModule = SwerveModule::new(4);




static mut TARGET_R: f32 = 0.;
static mut TARGET_R_DELTA: f32 = 2.;

#[tokio::main]
async fn main() {

    let mut window: PistonWindow = WindowSettings::new("Line Drawing", [SCREEN_SQUARE_SIZE, SCREEN_SQUARE_SIZE])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let assets = find_folder::Search::ParentsThenKids(3, 3)
        .for_folder("assets").unwrap();
    let ref font = assets.join("UbuntuMonoNerdFont-Regular.ttf");
    let mut glyphs = window.load_font(font).unwrap();


    fn draw_vertical_line(pos_x: f32, c: &Context, mut g: &mut G2d){
        let mut x = -pos_x % SCREEN_SQUARE_SIZE as f32;
        if x < 0. {x += SCREEN_SQUARE_SIZE as f32}

        line(color::GRAY, 1., [
            x as f64,
            0.,
            x as f64,
            SCREEN_SQUARE_SIZE as f64,
        ], c.transform, g);
    }

    fn draw_horizontal_line(pos_y: f32, c: &Context, mut g: &mut G2d){
        let mut y = -pos_y % SCREEN_SQUARE_SIZE as f32;
        if y < 0. {y += SCREEN_SQUARE_SIZE as f32}

        line(color::GRAY, 1., [
            0.,
            y as f64,
            SCREEN_SQUARE_SIZE as f64,
            y as f64,
        ], c.transform, g);
    }

    async unsafe fn update() {
        // let mut pid_x: pid::PIDController = pid::PIDController::new(
        //     2.,
        //     0.1,
        //     9.,
        //     0.
        // );
        //
        // let mut pid_y: pid::PIDController = pid::PIDController::new(
        //     2.,
        //     0.1,
        //     9.,
        //     0.
        // );

        let mut pid_rot: pid::PIDController = pid::PIDController::new(
            5.0,
            0.1,
            100.,
            0.
        );


        loop {
            if BUTTONS.key_w {
                VEL_Y -= (XY_ACCEL);
            } else if BUTTONS.key_s {
                VEL_Y += (XY_ACCEL);
            }

            if BUTTONS.key_a {
                VEL_X -= (XY_ACCEL);
            } else if BUTTONS.key_d {
                VEL_X += (XY_ACCEL);
            }

            if BUTTONS.key_q {
                TARGET_R -= TARGET_R_DELTA;
                pid_rot.set_setpoint(TARGET_R as f64);
            } else if BUTTONS.key_e {
                TARGET_R += TARGET_R_DELTA;
                pid_rot.set_setpoint(TARGET_R as f64);
            }





            VEL_R += ROT_ACCEL*pid_rot.update(POS_R as f64, 1.) as f32;


            VEL_R = f32::clamp(VEL_R, -MAX_ROT_VEL, MAX_ROT_VEL);

            VEL_X *= XY_FRICTION;
            VEL_Y *= XY_FRICTION;
            VEL_R *= ROT_FRICTION;

            // println!("{}", VEL_R);

            POS_X += VEL_X;
            POS_Y += VEL_Y;
            POS_R += VEL_R;



            W1.swerve_rot = POS_R;
            W2.swerve_rot = POS_R;
            W3.swerve_rot = POS_R;
            W4.swerve_rot = POS_R;


            //Obtain joystick data and define the heading
            let joyHeading = (get_heading(VEL_X, VEL_Y));
            let heading = joyHeading + POS_R;
            let speed = swerve_math::get_joystick_speed(VEL_X, VEL_Y);

            //Define the steering vector components and the max vector length
            let xr = VEL_R  * f32::cos(degrees_to_radians(45.)); // /2D normally
            let yr = VEL_R  * f32::sin(degrees_to_radians(45.)); // /2D normally

            //Calculate the vectors for all wheels
            let x = get_heading_x(heading);
            let y = get_heading_y(heading);


            W1.vec.set((x*speed + xr) * VECTOR_SCALE, (y*speed + yr) * VECTOR_SCALE);
            W2.vec.set((x*speed + xr) * VECTOR_SCALE, (y*speed - yr) * VECTOR_SCALE);
            W3.vec.set((x*speed - xr) * VECTOR_SCALE, (y*speed - yr) * VECTOR_SCALE);
            W4.vec.set((x*speed - xr) * VECTOR_SCALE, (y*speed + yr) * VECTOR_SCALE);

            println!("{}", W1.vec.x);

            W1.vec.rotate(POS_R);
            W2.vec.rotate(POS_R);
            W3.vec.rotate(POS_R);
            W4.vec.rotate(POS_R);

            thread::sleep(Duration::from_millis(10));
            // TIME += 1;
        }
    }

    let thread = tokio::spawn(unsafe { update() });

    while let Some(event) = window.next() {
        window.draw_2d(&event, |c, g, _d| unsafe {
            clear(color::BLACK, g);


            line(color::RED, 1., [
                SCREEN_SQUARE_SIZE as f64/2.,
                SCREEN_SQUARE_SIZE as f64/2.,
                (f32::cos(degrees_to_radians(TARGET_R as f32)) * SWERVE_CENTER_DISTANCE as f32 + (SCREEN_SQUARE_SIZE/2) as f32) as f64,
                (f32::sin(degrees_to_radians(TARGET_R as f32)) * SWERVE_CENTER_DISTANCE as f32 + (SCREEN_SQUARE_SIZE/2) as f32) as f64,
            ], c.transform, g);


            draw_horizontal_line(POS_Y, &c, g);
            draw_horizontal_line(POS_Y + SCREEN_SQUARE_SIZE as f32 /2., &c, g);
            draw_vertical_line(POS_X, &c, g);
            draw_vertical_line(POS_X + SCREEN_SQUARE_SIZE as f32 /2., &c, g);

            // if ROBO_RELATIVE {
            SwerveModule::draw(&mut W1, &c, g,
                               ((f32::cos(degrees_to_radians(POS_R - 45.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32,
                               ((f32::sin(degrees_to_radians(POS_R - 45.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32);

            SwerveModule::draw(&mut W2, &c, g,
                               ((f32::cos(degrees_to_radians(POS_R - 135.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32,
                               ((f32::sin(degrees_to_radians(POS_R - 135.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32);

            SwerveModule::draw(&mut W3, &c, g,
                               ((f32::cos(degrees_to_radians(POS_R + 135.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32,
                               ((f32::sin(degrees_to_radians(POS_R + 135.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32);

            SwerveModule::draw(&mut W4, &c, g,
                               ((f32::cos(degrees_to_radians(POS_R + 45.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32,
                               ((f32::sin(degrees_to_radians(POS_R + 45.)) * SWERVE_CENTER_DISTANCE as f32) + (SCREEN_SQUARE_SIZE/2) as f32) as u32);


            draw_text(&c, g, &mut glyphs, color::WHITE, 0., 15., format!("POS: [{},{}] VEL_XY: [{},{}]", POS_X, POS_Y, VEL_X, VEL_Y).as_str());
            draw_text(&c, g, &mut glyphs, color::WHITE, 0., 30., format!("ROT: {} VEL_ROT: {}", POS_R, VEL_R).as_str());

            Rectangle::new_border(color::WHITE, 1.0).draw(square(0.0, 0.0, SCREEN_SQUARE_SIZE as Scalar), &c.draw_state, c.transform, g);

            glyphs.factory.encoder.flush(_d);
        });

        unsafe {
            if let Some(Button::Keyboard(key)) = event.press_args() {
                if key.code() == 119 {
                    BUTTONS.key_w = true;
                } else if (key.code() == 97) {
                    BUTTONS.key_a = true;
                } else if (key.code() == 115) {
                    BUTTONS.key_s = true;
                } else if (key.code() == 100) {
                    BUTTONS.key_d = true;
                } else if (key.code() == 113) {
                    BUTTONS.key_q = true;
                } else if (key.code() == 101) {
                    BUTTONS.key_e = true;
                } else if key.code() == 114 {
                    // ROBO_RELATIVE = !ROBO_RELATIVE;
                } else if key.code() == 120 {
                    thread.abort();
                    break;
                }
            }

            if let Some(Button::Keyboard(key)) = event.release_args() {
                if key.code() == 119 {
                    BUTTONS.key_w = false;
                } else if (key.code() == 97) {
                    BUTTONS.key_a = false;
                } else if (key.code() == 115) {
                    BUTTONS.key_s = false;
                } else if (key.code() == 100) {
                    BUTTONS.key_d = false;
                } else if (key.code() == 113) {
                    BUTTONS.key_q = false;
                } else if (key.code() == 101) {
                    BUTTONS.key_e = false;
                }
            }
        }
    }
}

pub fn draw_text(
    ctx: &Context,
    graphics: &mut G2d,
    glyphs: &mut Glyphs,
    color: Color,
    x: f64,
    y: f64,
    text: &str,
) {
    text::Text::new_color(color, 15
    )
        .draw(
            text,
            glyphs,
            &ctx.draw_state,
            ctx.transform.trans(x,y),
            graphics,
        )
        .unwrap();
}
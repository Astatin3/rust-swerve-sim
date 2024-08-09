mod swerve_math;
mod swervemodule;

extern crate piston_window;

use piston_window::*;
use piston_window::rectangle::square;

fn main() {
    let mut window: PistonWindow = WindowSettings::new("Line Drawing", [512, 512])
        .exit_on_esc(true)
        .build()
        .unwrap();



    while let Some(event) = window.next() {
        if let Some(Button::Keyboard(key)) = event.press_args() {
            println!("Key pressed: {:?}", key);
            if key.code() == 113 {break;}
        }

        if let Some(Button::Mouse(key)) = event.press_args() {
            println!("Key pressed: {:?}", key);
        }

        window.draw_2d(&event, |c, g, _d| {
            clear(color::BLACK, g);

            // Ellipse::new_border(color::WHITE, 1.0).draw(
            //     rectangle::centered_square(10.0, 10.0, 50.0),
            //     &c.draw_state,
            //     c.transform,
            //     g,
            // );

            swervemodule::SwerveModule::draw(&c,);

            Rectangle::new_border(color::WHITE, 1.0).draw(square(0.0, 0.0, 512.0), &c.draw_state, c.transform, g);


        });
    }
}
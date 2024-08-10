use std::f32::consts::PI;

//https://github.com/Pantherbotics/SwerveSim/blob/master/src/main/java/com/pantherbotics/swervesim/util/Vector2d.java
#[derive(Clone, Copy)]
pub struct Vector2d {
    pub(crate) x: f32,
    pub(crate) y: f32
}
impl Vector2d {
    pub const fn create(mut self, x:f32, y:f32) {
        self.x = x;
        self.y = y;
    }

    pub fn set(&mut self, x:f32, y:f32) {
        self.x = x;
        self.y = y;
    }

    pub fn rotate(&mut self, angle: f32) {
        let mag = f32::sqrt(self.x*self.x+self.y*self.y);
        let ang = ((f32::atan2(self.y, self.x) + degrees_to_radians(angle)));
        self.x = f32::cos(ang)*mag;
        self.y = f32::sin(ang)*mag;
    }

    /**
     * Returns dot product of this vector with argument.
     *
     * @param vec Vector with which to perform dot product.
     * @return Dot product of this vector with argument.
     */
    pub fn dot(&mut self, vec: Vector2d) -> f32 {
        return self.x * vec.x + self.y * vec.y;
    }

    /**
     * Returns magnitude of vector.
     *
     * @return Magnitude of vector.
     */
    pub fn magnitude(self) -> f32 {
        return f32::sqrt(self.x * self.x + self.y * self.y);
    }

    /**
     * Returns scalar projection of this vector onto argument.
     *
     * @param vec Vector onto which to project this vector.
     * @return scalar projection of this vector onto argument.
     */
    pub fn scalar_project(&mut self, mut vec: Vector2d) -> f32 {
        let mag = vec.magnitude();
        return self.dot(vec) / mag;
    }
}

pub fn degrees_to_radians(degrees: f32) -> f32{
    return degrees * (PI / 180.0);
}

pub fn radians_to_degrees(radians: f32) -> f32{
    return radians * (180.0 / PI);
}




/**
 * I spent like half an hour figuring this out, don't try to figure it out just appreciate the results :)
 * 0 Degrees is straight forward, 90 degrees is to the right, 180 degrees is backwards, 270 degrees is to the left
 * Aka clockwise degrees and 0 is straight forward on the joystick :)
 * @param x the X value of a coordinate
 * @param y the Y value of a coordinate
 */
pub fn get_heading(x:f32, y:f32) -> f32 {
    if x == 0.0 && y == 0.0 { return 0.0; }

    let mut angle = (360. - ((f32::atan2(y, x)*180.0/PI) + 180.0)) - 90.0;
    if angle < 0. {
        angle = 270. + (90. - f32::abs(angle));
    }

    return angle;
}


/**
 * Used to re-obtain the X value of the point on a unit circle from an angle
 * The angles are in degrees from getHeading()
 * @param angle The angle (from getHeading()) to get the X value for
 */
pub fn get_heading_x(mut angle:f32) -> f32 {
    //Ensure values are [0, 360)
    while angle > 360. { angle -= 360.; }
    while angle < 0. { angle += 360.; }

    if angle >= 0. && angle <= 90. {
        return f32::cos(degrees_to_radians(90. - angle));
    }else if angle >= 90. && angle <= 270. {
        return f32::cos(-degrees_to_radians(angle - 90.));
    }else if angle >= 270. && angle <= 360. {
        return -f32::cos(degrees_to_radians(270. - angle));
    }
    return 0.;
}

/**
 * Used to re-obtain the Y value of the point on a unit circle from an angle
 * The angles are in degrees from getHeading()
 * @param angle The angle (from getHeading()) to get the Y value for
 */
pub fn get_heading_y(mut angle:f32) -> f32 {
    //Ensure values are [0, 360)
    while angle > 360. { angle -= 360.; }
    while angle < 0. { angle += 360.; }

    if angle >= 0. && angle <= 90. {
        return f32::sin(degrees_to_radians(90. - angle));
    }else if angle >= 90. && angle <= 270. {
        return f32::sin(-degrees_to_radians(angle - 90.));
    }else if angle >= 270. && angle <= 360. {
        return -f32::sin(degrees_to_radians(270. - angle));
    }
    return 0.;
}

/**
 * Takes a value and shifts it towards 0 by a specified amount
 * @param value the value to shift
 * @param shift the amount to shift it
 * @return the shifted value
 */
pub fn approach_zero(value:f32, shift:f32) -> f32 {
    if value >= 0. {
        return f32::max(0., value - shift);
    }else if value < 0. {
        return f32::min(0., value + shift);
    }
    return 0.;
}


/**
 * Returns a speed value from [-1, 1] based on joystick X and Y inputs
 * More critically it's snapped to the unit circle so X=1 Y=1 won't be sqrt(2)
 * @param X the X of a coordinate [-1, 1]
 * @param Y the Y of a coordinate [-1, 1]
 */
pub fn get_joystick_speed(y:f32, x:f32) -> f32 {
    let mut v: Vector2d = Vector2d {x:x, y:y};

    let angle = f32::atan2(v.x, v.y);
    let max_magnitude = if f32::abs(v.x) > f32::abs(v.y) {1. / f32::sin(angle)} else {1. / f32::cos(angle)};
    return f32::abs(Vector2d::magnitude(v) / max_magnitude);
}


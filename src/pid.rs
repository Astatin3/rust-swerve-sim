pub(crate) struct PIDController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    setpoint: f64,
    integral: f64,
    prev_error: f64,
}

impl PIDController {
    pub(crate) fn new(kp: f64, ki: f64, kd: f64, setpoint: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            setpoint,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    pub fn update(&mut self, process_variable: f64, dt: f64) -> f64 {
        let error = self.setpoint - process_variable;

        // Proportional term
        let p = self.kp * error;

        // Integral term
        self.integral += error * dt;
        let i = self.ki * self.integral;

        // Derivative term
        let d = self.kd * (error - self.prev_error) / dt;

        self.prev_error = error;

        // Calculate and return the control output
        p + i + d
    }

    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
    }
}
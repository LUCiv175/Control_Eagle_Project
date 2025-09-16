#pragma once

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd);

    // Compute steering angle given cross-track error and timestep
    double compute(double cte, double dt);

private:
    double Kp, Ki, Kd;
    double prev_error;
    double integral;
};

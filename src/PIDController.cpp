#include "PIDController.hpp"
#include <cmath>
#include <vector>

PIDController::PIDController(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), prev_error(0.0), integral(0.0) {}

double PIDController::compute(double cte, double dt) {
    integral += cte * dt;
    //clamping to manage the windups
    integral = std::clamp(integral, -1000.0, 1000.0);

    double derivative = (cte - prev_error) / dt;

    double delta = Kp * cte + Ki * integral + Kd * derivative;

    prev_error = cte;
    return -delta;
}


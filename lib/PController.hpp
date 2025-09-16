#pragma once

class PController {
public:
    PController(double Kp);

    // Compute steering angle given cross-track error
    double compute(double cte);

private:
    double Kp;
};

#include "BicycleModel.hpp"

BicycleModel::BicycleModel(double wheelbase) : L(wheelbase) {}

State BicycleModel::update(const State& s, double delta, double dt, double a) {
    State ns;
    ns.v = s.v + a * dt;        // velocity modified due to the acceleration (Level 1 Bonus)
    // Equations of motion
    ns.x = s.x + ns.v * std::cos(s.theta) * dt;
    ns.y = s.y + ns.v * std::sin(s.theta) * dt;
    ns.theta = s.theta + (ns.v / L) * std::tan(delta) * dt;
    return ns;
}

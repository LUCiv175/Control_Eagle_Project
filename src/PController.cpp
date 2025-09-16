#include "PController.hpp"

PController::PController(double Kp) : Kp(Kp) {}

double PController::compute(double cte) {
  // P control law: delta = -Kp * error
  return -Kp * cte;
}

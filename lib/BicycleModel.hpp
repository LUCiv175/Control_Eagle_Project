#include <cmath>

struct State {
    double x;
    double y;
    double theta;
    double v;
};

class BicycleModel {
public:
    BicycleModel(double wheelbase);

    State update(const State& s, double delta, double dt, double a);

private:
    double L;
};
#include <raylib.h>
#include "BicycleModel.hpp"
#include <vector>

int main() {
    InitWindow(800, 600, "Kinematic Bicycle Simulation");
    SetTargetFPS(60);

    BicycleModel car(2); // wheelbase
    State s{400, 300, 0, 100.0};// initial state
    double a = -50.0;
    double delta = 0.0;     // steering angle
    double dt;

    std::vector<Vector2> trajectory;

    while (!WindowShouldClose()) {
        dt = 0.01;

        // Update the state
        s = car.update(s, delta, dt, a);
        trajectory.push_back({(float)s.x, (float)s.y});

        // Draw
        BeginDrawing();
        ClearBackground(BLACK);

        // Draw trajectory
        for (auto point : trajectory) {
            DrawCircleV(point, 2, WHITE);
        }

        // Draw vehicle as a small rectangle
        DrawRectanglePro({(float)s.x, (float)s.y, 20, 10}, {10,5}, s.theta * 180.0f / 3.141592, RED);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

#include <raylib.h>
#include "BicycleModel.hpp"
#include <vector>
#include <random>
#include <cmath>

int main() {
    InitWindow(800, 600, "Kinematic Bicycle Simulation");
    SetTargetFPS(60);

    BicycleModel car(2.0); // wheelbase
    State s{0, 300, 0, 0.0}; // initial state
    double target_heading = 0.0; // straight path heading
    double target_y = 300.0;     // target lateral position
    double a = 50.0;           // acceleration (Level 1 BONUS)
    double delta = 0.0;     // steering angle
    double dt = 0.01;
    double Kp_heading = 1.0;  // proportional gain for heading
    double Kp_lateral = 0.02; // proportional gain for lateral error (Level 2 BONUS)

    std::vector<Vector2> trajectory;

    // Setup for disturbances
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> heading_noise(0.0, 0.02); // heading noise
    std::normal_distribution<double> lateral_drift(0.0, 1.0);  // lateral drift (Level 2 BONUS)

    while (!WindowShouldClose()) {
        // Add small disturbances (noise in heading axis)
        s.theta += heading_noise(gen);

        // BONUS: Add lateral drift (perpendicular to heading)
        double drift = lateral_drift(gen) * dt;
        s.x += drift * std::sin(s.theta);  // drift perpendicular to heading
        s.y -= drift * std::cos(s.theta);

        // Level 2: Proportional control law delta = Kp * e
        double heading_error = target_heading - s.theta;

        // BONUS: Add lateral error correction
        double lateral_error = target_y - s.y;

        // Combined control law (heading + lateral correction)
        delta = Kp_heading * heading_error + Kp_lateral * lateral_error;

        // Update the state
        s = car.update(s, delta, dt, a);
        trajectory.push_back({(float)s.x, (float)s.y});

        // Draw
        BeginDrawing();
        ClearBackground(BLACK);

        // Show controller info
        DrawText(("Steering: " + std::to_string(delta)).c_str(), 10, 10, 20, WHITE);
        DrawText(("Heading Error: " + std::to_string(heading_error)).c_str(), 10, 40, 20, WHITE);
        DrawText(("Lateral Error: " + std::to_string(lateral_error)).c_str(), 10, 70, 20, WHITE);

        // Draw target line (straight path)
        DrawLine(0, 300, 800, 300, GRAY);
        
        // Draw trajectory
        for (const auto& point : trajectory) {
            DrawCircleV(point, 2, GREEN);
        }

        // Draw vehicle
        DrawRectanglePro({(float)s.x, (float)s.y, 20, 10}, {10, 5}, 
                        s.theta * 180.0f / 3.141592f, RED);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
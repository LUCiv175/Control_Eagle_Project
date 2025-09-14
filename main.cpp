#include <raylib.h>
#include "BicycleModel.hpp"
#include <vector>
#include <random>
#include <cmath>

int main() {
    InitWindow(800, 600, "Kinematic Bicycle Simulation");
    SetTargetFPS(60);

    BicycleModel car(2.0); // wheelbase
    State s{0, 300, 0, 100.0}; // initial state
    double target_heading = 0.0; // straight path heading
    double a = 0.0;           // acceleration (Level 1 BONUS)
    double delta = 0.0;     // steering angle
    double dt = 0.01;
    double Kp_heading = 1.0;  // proportional gain for heading
    double Kp_lateral = 0.02; // proportional gain for lateral error (Level 2 BONUS)
    double Kp_steering = 0.0008; // proportional gain for steering

    std::vector<Vector2> trajectory;

    // Setup for disturbances
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> heading_noise(0.0, 0.02); // heading noise
    std::normal_distribution<double> lateral_drift(0.0, 1.0);  // lateral drift (Level 2 BONUS)

    // sinusoidal path
    std::vector<Vector2> path;
    for (double x = 0; x <= 800; x += 5.0) {
        double y = 300 + 200 * std::sin(0.01 * x);
        path.push_back({(float)x, (float)y});
    }


    while (!WindowShouldClose()) {
        // Add small disturbances (noise in heading axis)
        //s.theta += heading_noise(gen);

        // BONUS: Add lateral drift (perpendicular to heading)
        //double drift = lateral_drift(gen) * dt;
        //s.x += drift * std::sin(s.theta);  // drift perpendicular to heading
        //s.y -= drift * std::cos(s.theta);

        // Level 2: Proportional control law delta = Kp * e
        //double heading_error = target_heading - s.theta;

        // BONUS: Add lateral error correction
        //double lateral_error = target_y - s.y;

        //Level 3: P Controller for steering
        // Calculate the CTE for minimum distance
        double cte = 1e9; // Initialize with large value
        for (size_t i = 0; i < path.size() - 1; ++i) {
            Vector2 A = path.at(i);
            Vector2 B = path.at(i+1);

            // Segment vector
            double dx = B.x - A.x;
            double dy = B.y - A.y;
            double length2 = dx*dx + dy*dy;
            // Skip degenerate segments
            if (length2 == 0) continue;

            // Project vehicle position onto the line segment
            // t represents how far along the segment (0 = A, 1 = B)
            double t = ((s.x - A.x)*dx + (s.y - A.y)*dy) / length2;

            // Clamp t to [0,1] to stay within segment bounds
            t = std::clamp(t, 0.0, 1.0);

            // Calculate closest point on segment
            double closest_x = A.x + t * dx;
            double closest_y = A.y + t * dy;

            // Calculate distance vector from vehicle to closest point
            double vx = s.x - closest_x;
            double vy = s.y - closest_y;
            double dist = std::sqrt(vx*vx + vy*vy);

            // Determine which side of the path the vehicle is on
            // Using 2D cross product: positive = right side, negative = left side
            double side = dx*(s.y - A.y) - dy*(s.x - A.x);
            if (side < 0) dist = -dist;

            // Keep track of minimum absolute distance
            if (std::abs(dist) < std::abs(cte)) {
                cte = dist;
            }
        }


        // Combined control law (heading + lateral correction)
        //delta = Kp_heading * heading_error + Kp_lateral * lateral_error;
        delta = -Kp_steering * cte;

        // Update the state
        s = car.update(s, delta, dt, a);
        trajectory.push_back({(float)s.x, (float)s.y});

        // Draw
        BeginDrawing();
        ClearBackground(BLACK);

        // Show controller info
        DrawText(("Steering: " + std::to_string(delta)).c_str(), 10, 10, 20, WHITE);
        //DrawText(("Heading Error: " + std::to_string(heading_error)).c_str(), 10, 40, 20, WHITE);
        //DrawText(("Lateral Error: " + std::to_string(lateral_error)).c_str(), 10, 70, 20, WHITE);
        DrawText(("CTE: " + std::to_string(cte)).c_str(), 10, 40, 20, WHITE);


        // Draw target line (straight path)
        //DrawLine(0, 300, 800, 300, GRAY);
        
        // Draw trajectory
        for (const auto& point : trajectory) {
            DrawCircleV(point, 2, GREEN);
        }

        for (auto p : path) {
            DrawCircleV(p, 2, GRAY);
        }


        // Draw vehicle
        DrawRectanglePro({(float)s.x, (float)s.y, 20, 10}, {10, 5}, 
                        s.theta * 180.0f / 3.141592f, RED);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
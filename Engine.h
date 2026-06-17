#pragma once
#include <SFML/Graphics.hpp>
#include "PhysicsSystem.h"
#include <fstream>
#include <vector>
#include <string>

struct FrameSnapshot {
    float time;
    float pedalState;
    float finalTorque;
    float currRPM;
    float angularVelocity;
    float wheelForce;
    float carVelocityX;
    float accelerationG;
    std::string gripState;
};

class DataCollector {
private:
    std::vector<FrameSnapshot> logs;
    float totalTime = 0.f;

public:
    void recordFrame(float dt, const Motor& motor, const Wheel& wheel, sf::Vector2f carVel, sf::Vector2f carAccel, sf::Vector2f carDir) {
        totalTime += dt;
        
        FrameSnapshot snapshot;
        snapshot.time = totalTime;
        snapshot.pedalState = motor.pedalState;
        snapshot.finalTorque = motor.finalTorque;
        snapshot.currRPM = motor.currRPM;
        snapshot.angularVelocity = wheel.angularVelocity;
        
        // Calculate scalar speed along the X axis for simplicity
        snapshot.carVelocityX = carVel.x;
        
        // Calculate longitudinal G-force
        float longAccel = (carAccel.x * carDir.x) + (carAccel.y * carDir.y);
        snapshot.accelerationG = longAccel / 9.81f;

        // Determine grip state using your slip threshold
        float longVel = (carVel.x * carDir.x) + (carVel.y * carDir.y);
        float slipV = (wheel.angularVelocity * wheel.radius) - longVel;
        snapshot.gripState = (std::abs(slipV) < 0.2f) ? "GRIPPING" : "SLIPPING";
        
        logs.push_back(snapshot);
    }

    void exportToCSV(const std::string& filename) {
        std::ofstream file(filename, std::ios::out | std::ios::trunc);
        if (!file.is_open()) return;

        // Write CSV Header row
        file << "Time(s),PedalState,FinalTorque(Nm),EngineRPM,WheelAngVel(rad/s),CarVelocityX,Acceleration(g),Grip State\n";

        // Write frame records
        for (const auto& f : logs) {
            file << f.time << ","
                 << f.pedalState << ","
                 << f.finalTorque << ","
                 << f.currRPM << ","
                 << f.angularVelocity << ","
                 << f.carVelocityX*((18.0f)/(5.0f)) << ","
                 << f.accelerationG<<","
                 << f.gripState << "\n";
        }
        file.close();
    }
};
class Engine {
private:
    sf::RenderWindow window;
    PhysicsSystem physics;
    sf::Clock clock;
    sf::Font font;
    // Inside class Engine private members:
    DataCollector logger;
    RigidBody* playerVehicle; // Our test object pointer

    void handleInput(float dt);
    void render();

public:
    Engine();
    ~Engine();
    void run();
};
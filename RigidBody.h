#pragma once
#include <SFML/Graphics.hpp>
#include <cmath>

constexpr float g = 9.81f;

struct Motor {
    float maxTorque = 420.0f;
    float maxRPM = 13750.0f;
    float currRPM = 0.f;
    float currTorque = 0.f;
    float throttle = 0.f;
    float pedalState = 0.f;
    float finalTorque = 0.f;
    float gearRatio = 9.0f;

    sf::Vector2f motorUpdate(sf::Vector2f currvel,sf::Vector2f direc) {
        // Determine if we are moving forward or backward
        float longSpeed = currvel.x*direc.x+currvel.y*direc.y; 

        if (pedalState > 0.01f) {
            // Accelerating
            throttle = pedalState;

            if(currRPM>9000.0f){
                currTorque = maxTorque * (((9000.0f)*(9000.0f) / (currRPM*currRPM)))*((5.0f)/(9.0f));
            }
            else if (currRPM > 5000.0f) {
                currTorque = maxTorque * (((5000.0f)/ (currRPM)));
            } else {
                currTorque = maxTorque;
            }
            finalTorque = throttle * currTorque;
        } 
        else if (pedalState < -0.01f) {
            // Braking or Reversing
            if (longSpeed > 0.5f) { // Moving forward, so we BRAKE
                // Apply heavy negative torque (combining regen and mechanical brakes)
                finalTorque = -maxTorque * std::abs(pedalState) * 1.5f; 
            }
            else if(longSpeed>-0.5f){
                currvel.x=currvel.x-longSpeed*direc.x;
                currvel.y=currvel.y-longSpeed*direc.y;
                finalTorque=0.0f;
            }
            else { // Already stopped, so we REVERSE
                finalTorque = -maxTorque * std::abs(pedalState) * 0.5f; // Reverse is usually limited
            }
        } 
        else {
            // Coasting (One-Pedal Driving Regen)
            if (std::abs(longSpeed) > 0.5f) {
                finalTorque = -100.0f * (longSpeed / std::abs(longSpeed)); // Light regen drag opposing motion
            } else {
                finalTorque = 0.f;
            }
        }
        return currvel;
    }
};

struct Wheel {
    float radius = 0.356f;
    float staticFriction = 1.0f;
    float kineticFriction = 0.8f;
    float friction = 0.9f;
    float normalForce = 0.f;
    float angularVelocity = 0.f;
    
    // [FIX]: Set base inertia back to a realistic tire weight. 
    // The massive weight of the car will be added dynamically below.
    float inertia = 1.8f; 
    float angularAccelration = 0.f;

    // [FIX]: Passed in carDirection and carMass so the wheel knows what it is pushing
    float wheelUpdate(float torque, sf::Vector2f v, sf::Vector2f carDirection, float carMass, float dt) {
        float force;
        float longVel = (v.x * carDirection.x) + (v.y * carDirection.y);

        // 1. Calculate Slip Ratio
        float slipRatio = 0.f;
        if (std::abs(longVel) > 0.1f) {
            slipRatio = ((angularVelocity * radius) - longVel) / std::abs(longVel);
        } else {
            // Prevent divide-by-zero explosions at standstill
            slipRatio = ((angularVelocity * radius) - longVel) / 0.1f; 
        }

        // 2. Pacejka Curve (Dynamic Sliding Friction)
        float peakSlip = 0.1f;     
        float currentMu = (2.0f * staticFriction * peakSlip * std::abs(slipRatio)) / 
                        ((peakSlip * peakSlip) + (slipRatio * slipRatio));
                        
        if (std::abs(slipRatio) > peakSlip && currentMu < kineticFriction) {
            currentMu = kineticFriction; // Floor it at kinetic friction during big burnouts
        }

        // 3. THE FIX: Always check threshold against absolute max STATIC friction.
        // Use <= so the Traction Control System perfectly balances on the edge without triggering slip.
        if (std::abs(torque / radius) <= normalForce * staticFriction) {
            
            // --- GRIP STATE ---
            float effectiveInertia = inertia + (carMass * radius * radius);
            angularAccelration = torque / effectiveInertia;
            force = torque / radius;
            
            // Lock angular velocity to road speed
            angularVelocity = longVel / radius;
            friction = staticFriction; // Just for telemetry readout
            
        } else {
            
            // --- SLIP STATE ---
            float slipDirection = (slipRatio >= 0.f) ? 1.f : -1.f;
            
            // Use the Pacejka dynamic friction because we are sliding!
            force = -slipDirection * normalForce * currentMu; 
            
            angularAccelration = (torque - (force * radius)) / inertia;
            angularVelocity += angularAccelration * dt;
            friction = currentMu; // Just for telemetry readout
        }
        
        return force;
    }
};

struct RigidBody {
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    Motor motor;
    Wheel rear;
    float mass;
    float orientation; // In radians
    sf::Vector2f direction;
    
    sf::RectangleShape visualShape;

    RigidBody(sf::Vector2f startPos, sf::Vector2f size, float m) {
        position = startPos;
        mass = m;
        orientation = 0.f;
        velocity = sf::Vector2f(0.f, 0.f);
        acceleration = sf::Vector2f(0.f, 0.f);
        rear.normalForce = (m * g * 0.6f);

        float x = std::cosf(orientation);
        float y = std::sinf(orientation);
        direction = sf::Vector2f(x, y);

        visualShape.setSize(size);
        visualShape.setOrigin(size.x / 2.f, size.y / 2.f); 
        visualShape.setFillColor(sf::Color::Green);
    }

    void bodyUpdate(float dt, sf::Vector2f force){
        acceleration = force / mass;
        velocity += acceleration * dt;
        position += velocity * dt;
    }
    
    void updateVisual() {
        visualShape.setPosition(position);
        // [FIX]: Converted radians to degrees for SFML rendering
        visualShape.setRotation(orientation * 180.f / 3.14159f); 
    }
};
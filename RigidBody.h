#pragma once
#include <SFML/Graphics.hpp>
#include <cmath>

constexpr float g = 9.81f;

struct Motor {
    float maxTorque = 500.0f;
    float maxRPM = 18000.0f;
    float currRPM = 0.f;
    float currTorque = 0.f;
    float throttle = 0.f;
    float pedalState = 0.f;
    float finalTorque = 0.f;
    float gearRatio = 9.0f;

    void motorUpdate(sf::Vector2f currvel,sf::Vector2f direc) {
        // Determine if we are moving forward or backward
        float longSpeed = currvel.x*direc.x+currvel.y*direc.y; 

        if (pedalState > 0.05f) {
            // Accelerating
            throttle = pedalState * pedalState*pedalState;
            if (currRPM > 6000.0f) {
                currTorque = maxTorque * (1.0f + (6000.0f / maxRPM) - (4.0f / 3.0f) * (currRPM / maxRPM));
            } else {
                currTorque = maxTorque;
            }
            finalTorque = throttle * currTorque;
        } 
        else if (pedalState < -0.05f) {
            // Braking or Reversing
            if (longSpeed > 0.5f) { // Moving forward, so we BRAKE
                // Apply heavy negative torque (combining regen and mechanical brakes)
                finalTorque = -maxTorque * std::abs(pedalState) * 1.5f; 
            } else { // Already stopped, so we REVERSE
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
    }
};

struct Wheel {
    float radius = 0.355f;
    float staticFriction = 0.9f;
    float kineticFriction = 0.7f;
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
        
        // [FIX]: Use the Dot Product to find true forward/backward speed instead of absolute magnitude.
        float longVel = (v.x * carDirection.x) + (v.y * carDirection.y);
        
        float slipV = (angularVelocity * radius) - longVel;
        friction = (std::abs(slipV) < 0.2f) ? staticFriction : kineticFriction;
        
        float spinDirection = (torque >= 0.f) ? 1.f : -1.f;

        if (std::abs(torque / radius) < normalForce * friction) {
            // [FIX]: GRIP STATE - Calculate Effective Inertia using the car's full mass
            float effectiveInertia = inertia + (carMass * radius * radius);
            angularAccelration = torque / effectiveInertia;
            force = torque / radius;
            
            // [FIX]: Lock the wheel rotation directly to the car's physical speed so it doesn't free-spin
            angularVelocity = longVel / radius;
        } else {
            // [FIX]: SLIP STATE - The wheel breaks free and only fights its own light inertia
            force = spinDirection * normalForce * friction;
            angularAccelration = (torque - (force * radius)) / inertia;
            angularVelocity += angularAccelration * dt;
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
        rear.normalForce = (m * g) / 2.f;

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
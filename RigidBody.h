#pragma once
#include <SFML/Graphics.hpp>
constexpr float g = 9.81f;
struct Motor {
    float maxTorque=420.0f;
    float maxRPM=14000.0f;
    float currRPM;
    float currTorque;
    float throttle;
    float pedalState;
    float finalTorque;
    float gearRatio=9.5f;
    void motorUpdate() {
        throttle=pedalState*pedalState;
        currTorque=maxTorque*(1-(currRPM/maxTorque));
        finalTorque=throttle*currTorque;
    }
};

struct Wheel {
    float radius=0.33;
    float staticFriction=0.9;
    float kineticFriction=0.7;
    float friction;
    float normalForce;
    float angularVelocity;
    float inertia=0.9;
    float angularAccelration;
    float wheelUpdate(float torque, float velocity) {
        float force;
        if(angularVelocity*radius>(0.9*velocity) && angularVelocity*radius<(1.1*velocity)){
            friction=staticFriction;
        }
        else friction=kineticFriction;
        if(torque<normalForce*friction*radius){
            angularAccelration=0;
            force=torque;
        }
        else{
            angularAccelration=(torque-normalForce*friction*radius)/inertia;
            
        }
    }
};

struct RigidBody {
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    sf::Vector2f totalForce;
    Motor motor;
    Wheel rear;
    float mass;
    float orientation; // In degrees
    
    sf::RectangleShape visualShape;

    RigidBody(sf::Vector2f startPos, sf::Vector2f size, float m) {
        position = startPos;
        mass=m;
        orientation = 0.f;
        velocity = sf::Vector2f(0.f, 0.f);
        acceleration = sf::Vector2f(0.f, 0.f);
        totalForce = sf::Vector2f(0.f, 0.f);
        rear.normalForce=(m*g)/4;

        visualShape.setSize(size);
        visualShape.setOrigin(size.x / 2.f, size.y / 2.f); 
        visualShape.setFillColor(sf::Color::Green);
    }

    void applyForce(sf::Vector2f force) {
        totalForce += force;
    }

    void updateVisual() {
        visualShape.setPosition(position);
        visualShape.setRotation(orientation);
    }
};
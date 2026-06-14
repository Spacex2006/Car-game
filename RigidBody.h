#pragma once
#include <SFML/Graphics.hpp>

struct RigidBody {
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    sf::Vector2f totalForce;
    
    float mass;
    float orientation; // In degrees
    
    sf::RectangleShape visualShape;

    RigidBody(sf::Vector2f startPos, sf::Vector2f size, float m) {
        position = startPos;
        mass = m;
        orientation = 0.f;
        velocity = sf::Vector2f(0.f, 0.f);
        acceleration = sf::Vector2f(0.f, 0.f);
        totalForce = sf::Vector2f(0.f, 0.f);

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
#pragma once
#include <SFML/Graphics.hpp>
#include "PhysicsSystem.h"

class Engine {
private:
    sf::RenderWindow window;
    PhysicsSystem physics;
    sf::Clock clock;
    
    RigidBody* playerVehicle; // Our test object pointer

    void handleInput(float dt);
    void render();

public:
    Engine();
    ~Engine();
    void run();
};
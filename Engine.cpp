#include "Engine.h"
constexpr float PI = 3.14f;

Engine::Engine() : window(sf::VideoMode(800, 600), "Multi-File Custom Engine") {
    window.setFramerateLimit(60);
    
    // Create a 1200kg test car at the center of the screen
    playerVehicle = new RigidBody(sf::Vector2f(400.f, 300.f), sf::Vector2f(20.f, 10.f), 1800);
    physics.registerBody(playerVehicle);
}

Engine::~Engine() {
    delete playerVehicle;
}

void Engine::run() {
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        handleInput(dt);
        physics.update(dt);
        render();
    }
}

void Engine::handleInput(float dt) {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
    }

    // Apply forces based on keyboard state
    float chargeRate = 2.0f; // Takes 0.5 seconds to fully depress
    float decayRate = 4.0f;  // Takes 0.25 seconds to fully lift foot

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
        playerVehicle->motor.pedalState += chargeRate * dt;
        // Hard cap at 1.0 so it never goes higher
        if (playerVehicle->motor.pedalState > 1.0f) {
            playerVehicle->motor.pedalState = 1.0f;
        }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
        playerVehicle->motor.pedalState -= decayRate * dt;
        // Hard cap at 1.0 so it never goes higher
        if (playerVehicle->motor.pedalState > 0.0f) {
            playerVehicle->motor.pedalState = 0.0f;
        }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
        playerVehicle->motor.pedalState -= decayRate * dt;
        // Hard cap at 1.0 so it never goes higher
        if (playerVehicle->motor.pedalState > 0.0f) {
            playerVehicle->motor.pedalState = 0.0f;
        }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        playerVehicle->motor.pedalState -= decayRate * dt;
        // Hard cap at 1.0 so it never goes higher
        if (playerVehicle->motor.pedalState > 0.0f) {
            playerVehicle->motor.pedalState = 0.0f;
        }
    }
    playerVehicle->motor.currRPM=playerVehicle->rear.angularVelocity*(30/PI)*(playerVehicle->motor.gearRatio);
    playerVehicle->motor.motorUpdate();
}

void Engine::render() {
    window.clear(sf::Color(30, 30, 30));
    
    // Draw our vehicle using its internal visual representation
    window.draw(playerVehicle->visualShape);
    
    window.display();
}
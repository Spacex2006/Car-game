#include "Engine.h"

Engine::Engine() : window(sf::VideoMode(800, 600), "Multi-File Custom Engine") {
    window.setFramerateLimit(60);
    
    // Create a 1200kg test car at the center of the screen
    playerVehicle = new RigidBody(sf::Vector2f(400.f, 300.f), sf::Vector2f(40.f, 20.f), 1200.f);
    physics.registerBody(playerVehicle);
}

Engine::~Engine() {
    delete playerVehicle;
}

void Engine::run() {
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        handleInput();
        physics.update(dt);
        render();
    }
}

void Engine::handleInput() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
    }

    // Apply forces based on keyboard state
    float engineThrust = 600000.f; 

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
        playerVehicle->applyForce(sf::Vector2f(engineThrust, 0.f)); // Push Right
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
        playerVehicle->applyForce(sf::Vector2f(-engineThrust, 0.f)); // Push Left
    }
}

void Engine::render() {
    window.clear(sf::Color(30, 30, 30));
    
    // Draw our vehicle using its internal visual representation
    window.draw(playerVehicle->visualShape);
    
    window.display();
}
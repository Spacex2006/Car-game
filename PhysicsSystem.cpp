#include "PhysicsSystem.h"

void PhysicsSystem::registerBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsSystem::update(float dt) {
    for (auto* body : bodies) {
        if (body->mass <= 0.f) continue; // Ignore static objects

        // Basic Physics Pipeline (A = F / M)
        body->acceleration = body->totalForce / body->mass;
        body->velocity += body->acceleration * dt;
        body->position += body->velocity * dt;

        // Reset forces for the next frame
        body->totalForce = sf::Vector2f(0.f, 0.f);

        // Sync visual transform
        body->updateVisual();
    }

    resolveCollisions();
}

void PhysicsSystem::resolveCollisions() {
    // This is where you will write your custom collision algorithms later!
}
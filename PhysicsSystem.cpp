#include "PhysicsSystem.h"

void PhysicsSystem::registerBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsSystem::update(float dt) {
    for (auto* body : bodies) {
        if (body->mass <= 0.f) continue; //Ignore static objects
        //Basic Physics
        

        // Sync visual transform
        body->updateVisual();
    }

    resolveCollisions();
}

void PhysicsSystem::resolveCollisions() {
    // This is where you will write your custom collision algorithms later!
}
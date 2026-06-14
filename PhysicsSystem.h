#pragma once
#include "RigidBody.h"
#include <vector>

class PhysicsSystem {
private:
    std::vector<RigidBody*> bodies;
    void resolveCollisions(); // Internal collision logic hook

public:
    void registerBody(RigidBody* body);
    void update(float dt);
};
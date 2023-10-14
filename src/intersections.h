#pragma once

#include "physics_body.h"

namespace Intersection {

    inline bool sphere_sphere(const sPhysicsBody &b1, 
                              const sPhysicsBody &b2) {
        const glm::vec3 b1_to_b2 = b2.position - b1.position;
        const float radius_sum = b1.scale.x + b2.scale.x;

        return glm::length(b1_to_b2) < radius_sum;
    }
};
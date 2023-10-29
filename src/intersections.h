#pragma once

#include "physics_body.h"

namespace Intersection {

    inline bool sphere_sphere(const sPhysicsBody &b1, 
                              const sPhysicsBody &b2, 
                              sCollisionManifold *manifest) {
        const glm::vec3 b1_to_b2 = b2.position - b1.position;
        const float radius_sum = b1.scale.x + b2.scale.x;

        manifest->normal = glm::normalize(b1_to_b2);

        manifest->point_in_body1_world = b1.position + manifest->normal * b1.scale.x;
        manifest->point_in_body2_world = b2.position - manifest->normal * b2.scale.x;

        return glm::length(b1_to_b2) < radius_sum;
    }
};
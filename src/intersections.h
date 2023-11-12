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


    inline bool ray_sphere(const glm::vec3 &ray_origin, 
                           const glm::vec3 &ray_dir, 
                           const glm::vec3 &sphere_center, 
                           const float sphere_radius, 
                           float *t1, 
                           float *t2) {
        const glm::vec3 sphere_to_ray = sphere_center - ray_origin;
        const float ray_dir_2 = glm::dot(ray_dir, ray_dir);
        const float facing = glm::dot(sphere_to_ray, ray_dir);
        const float distance_to_intersection_point_2 = glm::dot(sphere_to_ray, sphere_to_ray) - sphere_radius * sphere_radius;

        const float delta = facing * facing - ray_dir_2 * distance_to_intersection_point_2;

        if (delta < 0.0f) {
            return false;
        }

        const float inv_ray_dir_2 = 1.0f / ray_dir_2;
        const float delta_sqrt = sqrtf(delta);

        *t1 = inv_ray_dir_2 * (facing - delta_sqrt);
        *t2 = inv_ray_dir_2 * (facing + delta_sqrt);

        return true;
    }

    inline bool sphere_sphere_ccd(const sPhysicsBody &b1,
                                  const sBodySpeed   &speeds1,
                                  const sPhysicsBody &b2, 
                                  const sBodySpeed   &speeds2,
                                  const float delta,
                                  float *ToI, 
                                  glm::vec3 *point_on_body1, 
                                  glm::vec3 *point_on_body2) {
        //
        const glm::vec3 relative_velocity = speeds1.linear_velocity - speeds2.linear_velocity;

        const glm::vec3 start_point_1 = b1.position;
        const glm::vec3 end_point_1 = b1.position + relative_velocity * delta;
        const glm::vec3 ray_dir = end_point_1 - start_point_1;

        float t0 = 0.0f, t1 = 0.0f;
        if (glm::length(ray_dir) < 0.001f) {
            // Ray is too short
            glm::vec3 distance = b2.position - b1.position;
            float radius = b1.scale.x + b2.scale.x + 0.001f;
            if (glm::length(distance) > radius) {
                return false;
            }
        } else if (!ray_sphere(b1.position, ray_dir, b2.position, b1.scale.x + b2.scale.x, &t0, &t1)) {
            return false;
        }

        // From (0,1) to (0, dt) range
        t0 *= delta;
        t1 *= delta;

        // If this is true, the collision is in the past, not in this frame
        if (t1 < 0.0f) {
            return false;
        }

        // The Time of Impact
        *ToI = (t0 < 0.0f) ? 0.0f : t0;

        // Collision is far in the future
        if (*ToI > delta) {
            return false;
        }

        glm::vec3 new_pos1 = b1.position + speeds1.linear_velocity * *ToI;
        glm::vec3 new_pos2 = b2.position + speeds2.linear_velocity * *ToI;

        glm::vec3 dist = glm::normalize(new_pos2 - new_pos1);

        *point_on_body1 = new_pos1 + dist * b1.scale.x;
        *point_on_body2 = new_pos2 - dist * b2.scale.x;

        return true;
    }

};
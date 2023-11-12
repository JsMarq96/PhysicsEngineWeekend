#pragma once

#include <cstdint>
#include <glm/fwd.hpp>
#include <glm/gtc/quaternion.hpp>
#include "glm/ext/quaternion_geometric.hpp"
#include "glm/matrix.hpp"
#include "render/mesh.h"
#include "render/camera.h"
#include "utils.h"
#include "physics_body.h"
#include "physics_sim_parameters.h"

#define BODY_TOTAL_SIZE 254u
#define MANIFOLD_COUNT (BODY_TOTAL_SIZE * BODY_TOTAL_SIZE)

struct sPhysicsWorld {
    uint32_t                    width = 0u;
    uint32_t                    height = 0u;

    sPhysicsBody                bodies[BODY_TOTAL_SIZE];
    sBodySpeed                  speeds[BODY_TOTAL_SIZE];
    glm::vec3                   local_center_of_mass[BODY_TOTAL_SIZE];
    glm::mat3                   local_inertia_tensors[BODY_TOTAL_SIZE];
    bool                        is_body_enabled[BODY_TOTAL_SIZE];

    uint16_t                    collision_count = 0u;
    sCollisionManifold          collisions_in_frame[MANIFOLD_COUNT];
    uint16_t                    collision_index_in_frame[MANIFOLD_COUNT];

    float                       camera_angle = 0.0f;
    Renderer::sCamera           camera;
    Renderer::sMeshRenderer     sphere_renderer;
    Renderer::sMeshRenderer     debug_renderer;

    bool start = false;
    
    void init();
    void update(const float delta);
    void update_body(const uint8_t index, const float delta);
    void physics_update(const float delta);
    void resolve_collision(const sCollisionManifold &manifold);
    uint8_t add_body(const sPhysicsBody &new_body);
    void render();
    void clean();
    void window_resized(const float height, const float width);

    inline glm::mat3 get_inverse_inertia_tensor_in_local(const uint8_t index) {
        return glm::inverse(local_inertia_tensors[index]) * bodies[index].inv_mass;
    }

    inline glm::mat3 get_inverse_inertia_tensor_in_world(const uint8_t index) {
        glm::mat3 orientation = glm::toMat3(bodies[index].orientation);

        return orientation * get_inverse_inertia_tensor_in_local(index) * glm::transpose(orientation);
    }

    inline glm::vec3 get_center_of_mass_in_world(const uint8_t index) {
        return bodies[index].position + (bodies[index].orientation * local_center_of_mass[index]);
    }

    inline void apply_linear_impulse(const uint8_t i, const glm::vec3 &J) {
        float inv_mass = bodies[i].inv_mass;
        if (!is_body_enabled[i] || inv_mass == 0.0f){
            return;
        }

        // Linear impulse
        speeds[i].linear_velocity += J * inv_mass;
    }

    inline void apply_impulse(const uint8_t i, const glm::vec3 &J, const glm::vec3 &impulse_point) {
        float inv_mass = bodies[i].inv_mass;
        if (!is_body_enabled[i] || inv_mass == 0.0f){
            return;
        }

        // Linear impulse
        speeds[i].linear_velocity += J * inv_mass;

        // Angular impulse
        glm::vec3 r = impulse_point - get_center_of_mass_in_world(i);
        glm::vec3 J_angular = glm::cross(r, J);
        speeds[i].angular_velocity += get_inverse_inertia_tensor_in_world(i) * J_angular;

        // Clamp angular speed
        if (glm::length(speeds[i].angular_velocity) > MAX_ANGULAR_SPEED) {
            speeds[i].angular_velocity = glm::normalize(speeds[i].angular_velocity) * MAX_ANGULAR_SPEED;
        }
    }
};
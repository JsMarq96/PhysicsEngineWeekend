#pragma once

#include <cstdint>
#include <glm/fwd.hpp>
#include <glm/gtc/quaternion.hpp>
#include "render/mesh.h"
#include "render/camera.h"
#include "utils.h"
#include "physics_body.h"

#define BODY_TOTAL_SIZE 254u
#define MANIFOLD_COUNT (BODY_TOTAL_SIZE * BODY_TOTAL_SIZE)

struct sPhysicsWorld {
    uint32_t                    width = 0u;
    uint32_t                    height = 0u;

    sPhysicsBody                bodies[BODY_TOTAL_SIZE];
    sBodySpeed                  speeds[BODY_TOTAL_SIZE];
    bool                        is_body_enabled[BODY_TOTAL_SIZE];

    uint16_t                    collision_count = 0u;
    sCollisionManifold          collisions_in_frame[MANIFOLD_COUNT];

    float                       camera_angle = 0.0f;
    Renderer::sCamera           camera;
    Renderer::sMeshRenderer     sphere_renderer;
    
    void init();
    void update(const float delta);
    void physics_update(const float delta);
    void resolve_collision(const sCollisionManifold &manifold);
    void render();
    void clean();
    void window_resized(const float height, const float width);

    inline void apply_impulse(const uint32_t i, const glm::vec3 &J) {
        float inv_mass = bodies[i].inv_mass;
        if (!is_body_enabled[i] || inv_mass == 0.0f){
            return;
        }

        speeds[i].linear_velocity += J * inv_mass;
    }

    inline uint8_t add_body(const sPhysicsBody &new_body) {
        uint8_t index = 0u;
        for(; index < BODY_TOTAL_SIZE; index++) {
            if (!is_body_enabled[index]) {
                break;
            }
        }

        is_body_enabled[index] = true;
        bodies[index] = new_body;

        return index;
    }
};
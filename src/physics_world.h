#pragma once

#include <glm/fwd.hpp>
#include <glm/gtc/quaternion.hpp>
#include "render/mesh.h"
#include "render/camera.h"
#include "utils.h"
#include <cstdint>

#define BODY_TOTAL_SIZE 200u

enum eColliderType : uint8_t {
    SPHERE_COLLIDER = 0u,
    COLLIDER_COUNT
};

struct sPhysicsBody {
    eColliderType      type = SPHERE_COLLIDER;
    glm::vec3          position = {0.0f, 0.0f, 0.0f};
    glm::vec3          scale = {1.0f, 1.0f, 1.0f};
    glm::quat          orientation = {1.0f, 0.0f, 0.0f, 0.0f};

    glm::vec4         color = {0.0f, 0.0f, 0.0f, 1.0f};
};

struct sPhysicsWorld {
    uint32_t width = 0u;
    uint32_t height = 0u;

    sPhysicsBody bodies[BODY_TOTAL_SIZE];
    bool         is_body_enabled[BODY_TOTAL_SIZE];

    float camera_angle = 0.0f;
    Renderer::sCamera    camera;
    Renderer::sMeshRenderer sphere_renderer;
    
    void init();
    void update(const float delta);
    void render();
    void clean();
    void window_resized(const float height, const float width);

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
#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

enum eColliderType : uint8_t {
    SPHERE_COLLIDER = 0u,
    COLLIDER_COUNT
};

struct sCollisionManifold {
    glm::vec3 point_in_body1_world;
    glm::vec3 point_in_body2_world;

    glm::vec3 normal;
    float separation_distance;
    float time_of_impact;

    uint8_t body1_index;
    uint8_t body2_index;
};


struct sPhysicsBody {
    eColliderType      type = SPHERE_COLLIDER;
    float              inv_mass = 0.0f;
    float              elasticity = 0.0f;
    float              friction = 0.0f;
    glm::vec3          position = {0.0f, 0.0f, 0.0f};
    glm::vec3          scale = {1.0f, 1.0f, 1.0f};
    glm::quat          orientation = {1.0f, 0.0f, 0.0f, 0.0f};

    glm::vec4          color = {0.0f, 0.0f, 0.0f, 1.0f};


    inline glm::vec3 get_center_of_mass() const {
        if (type == SPHERE_COLLIDER) {
            return glm::vec3(0.0f);
        }
        return glm::vec3(0.0f);
    }

    inline glm::vec3 get_world_center_of_mass() const {
        return position + orientation * get_center_of_mass();
    }

    inline glm::vec3 world_to_body_space(const glm::vec3 point) const {
        return glm::inverse(orientation) * (point - get_world_center_of_mass());
    }

    inline glm::vec3 body_to_world_space(const glm::vec3 point) const {
        return get_world_center_of_mass() + orientation * point;
    }
};

struct sBodySpeed {
    glm::vec3       linear_velocity = {0.0f, 0.0f, 0.0f};
    glm::vec3       angular_velocity = {0.0f, 0.0f, 0.0f};
};
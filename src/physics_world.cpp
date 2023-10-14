#include "physics_world.h"
#include "glm/matrix.hpp"

#include <imgui.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

glm::vec3 rotate_arround(const glm::vec3  &pos, 
                         const glm::vec3  &center, 
                         const float angle);

void sPhysicsWorld::init() {
    sphere_renderer.create_from_file("resources/sphere.obj");

    camera.look_at({0.0f, 0.0f, 0.0f}, {2.0f, 1.0f, 2.0f});

    // Clean memory for the bodies
    memset(is_body_enabled, false, sizeof(is_body_enabled));
    memset(speeds, 0, sizeof(speeds));
    memset(bodies, 0, sizeof(bodies));

    add_body({  .type = SPHERE_COLLIDER,
                .position = {0.0f, 0.0f, 0.0f},
                .scale = {1.0f, 1.0f, 1.0f},
                .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
                .color = {0.05f, 0.05f, 1.05f, 1.0f}
            });
}
void sPhysicsWorld::update(const float delta) {
    // ImGUI & camera controls
    {
        ImGui::Begin("Camera control");

        bool camera_moved = false;
        camera_moved |= ImGui::SliderFloat3("Camera look at", (float*)&camera.center, -5.0f, 5.0f);
        camera_moved |= ImGui::SliderFloat("Camera rotation", &camera_angle, 0.01f, 20.14f);
        
        camera.position = rotate_arround(camera.position, glm::vec3(0.0f), glm::radians(camera_angle));

        if (camera_moved) {
            camera.look_at(camera.center, camera.position);
        }
        ImGui::End();
    }
    
}

void sPhysicsWorld::physics_update(const float delta) {
    // Integrate accelerations into velocity
    for(uint8_t i = 0u; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i]) {
            continue;
        }

        speeds[i].linear_velocity += glm::vec3(0.0f, -10.0f, 0.0f) * delta;
    }

    // Integrate velocity into position
    for(uint8_t i = 0u; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i]) {
            continue;
        }

        bodies[i].position += speeds[i].linear_velocity * delta;
    }
}

void sPhysicsWorld::render() {

    glm::mat4 vp_mat;
    camera.get_perspective_viewprojection_matrix(90.0f,
                                                 10.0f,
                                                 0.1f,
                                                 (float) width / height,
                                                 &vp_mat);
    glm::mat4 model;
    // Instances to render
    for(uint8_t i = 0; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i]) {
            continue;
        }

        sPhysicsBody &curr_body = bodies[i];
        //First scale
        model = glm::scale(glm::mat4(1.0f), curr_body.scale);
        model = glm::toMat4(curr_body.orientation) * model;
        model = glm::translate(model, curr_body.position);

        if (curr_body.type == SPHERE_COLLIDER) {
            sphere_renderer.add(model, curr_body.color);
        }
        
    }

    sphere_renderer.render(vp_mat);
}

void sPhysicsWorld::clean() {
    sphere_renderer.clean();
}

void sPhysicsWorld::window_resized(const float h, const float w) {
    width = w;
    height = h;
}


glm::vec3 rotate_arround(const glm::vec3  &pos, 
                         const glm::vec3  &center, 
                         const float angle) {
  float x = pos.x - center.x, z = pos.z - center.z;
  float s = glm::sin(angle), c = glm::cos(angle);

  float nx = pos.x * c - pos.z * s;
  float nz = pos.x * s + pos.z * c;

  return glm::vec3(nx + center.x, pos.y, nz + center.z);
}
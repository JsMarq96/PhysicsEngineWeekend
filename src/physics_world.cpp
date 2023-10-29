#include "physics_world.h"

#include "glm/gtx/norm.hpp"
#include "glm/matrix.hpp"
#include <imgui.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "intersections.h"

glm::vec3 rotate_arround(const glm::vec3  &pos, 
                         const glm::vec3  &center, 
                         const float angle);

void sPhysicsWorld::init() {
    sphere_renderer.create_from_file("resources/sphere.obj");

    camera.look_at({0.0f, 0.0f, 0.0f}, {5.0f, 2.0f, 5.0f});

    // Clean memory for the bodies
    memset(is_body_enabled, false, sizeof(is_body_enabled));
    memset(speeds, 0, sizeof(speeds));
    memset(bodies, 0, sizeof(bodies));

    add_body({  .type = SPHERE_COLLIDER,
                .inv_mass = 1.0f / 5.0f,
                .elasticity = 0.5f,
                .position = {0.0f, 3.0f, 0.5f},
                .scale = {1.0f, 1.0f, 1.0f},
                .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
                .color = {0.05f, 0.05f, 0.05f, 1.0f}
            });

    add_body({  .type = SPHERE_COLLIDER,
                .inv_mass = 0.0f, // Static body
                .elasticity = 1.0f,
                .position = {0.0f, -100.0f, 0.0f},
                .scale = {100.0f, 100.0f, 100.0f},
                .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
                .color = {0.05f, 0.05f, 1.05f, 1.0f}
            });
}
void sPhysicsWorld::update(const float delta) {
    // ImGUI & camera controls
    {
        ImGui::Begin("Camera control");

        bool camera_moved = false;
        ImGui::SliderFloat3("body pos", (float*)&bodies[1].position, -10.0f, 10.0f);
        camera_moved |= ImGui::SliderFloat3("Camera look at", (float*)&camera.center, -10.0f, 10.0f);
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

        apply_impulse(i, glm::vec3(0.0f, -10.0f, 0.0f) * (1.0f / bodies[i].inv_mass) * delta);
    }

    // Collision detection
    // Only skip tests in one direction with static objects
    collision_count = 0u;
    for(uint8_t i = 0u; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i] || bodies[i].inv_mass == 0.0f) {
            continue;
        }
        for(uint8_t j = i + 1u; j < BODY_TOTAL_SIZE; j++) {
            if (!is_body_enabled[j]) {
                continue;
            }

            if (Intersection::sphere_sphere(bodies[i], bodies[j], &collisions_in_frame[collision_count])) {
                collisions_in_frame[collision_count].body1_index = i;
                collisions_in_frame[collision_count].body2_index = j;
                resolve_collision(collisions_in_frame[collision_count]);
                collision_count++;
            }
        }
    }

    // Integrate velocity into position
    for(uint8_t i = 0u; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i]) {
            continue;
        }

        bodies[i].position += speeds[i].linear_velocity * delta;
    }
}

void sPhysicsWorld::resolve_collision(const sCollisionManifold &manifold) {
    const float elasticity = bodies[manifold.body1_index].elasticity * bodies[manifold.body2_index].elasticity; 

    // Collision impulse
    const glm::vec3 linear_velocity_delta = speeds[manifold.body1_index].linear_velocity - speeds[manifold.body2_index].linear_velocity;
    const float impulse = -(1.0f + elasticity) * glm::dot(linear_velocity_delta, manifold.normal) / (bodies[manifold.body1_index].inv_mass + bodies[manifold.body2_index].inv_mass);

    apply_impulse(manifold.body1_index, manifold.normal * impulse);
    apply_impulse(manifold.body2_index, manifold.normal * impulse * 1.0f);


    // Manage interpenetration
    const float margin_body1 = bodies[manifold.body1_index].inv_mass / (bodies[manifold.body1_index].inv_mass + bodies[manifold.body2_index].inv_mass);
    const float margin_body2 = bodies[manifold.body2_index].inv_mass / (bodies[manifold.body1_index].inv_mass + bodies[manifold.body2_index].inv_mass);
    
    const glm::vec3 contact_distance = manifold.point_in_body2_world - manifold.point_in_body1_world;
    
    bodies[manifold.body1_index].position += contact_distance * margin_body1;
    bodies[manifold.body2_index].position -= contact_distance * margin_body2;
}

inline uint8_t sPhysicsWorld::add_body(const sPhysicsBody &new_body) {
    uint8_t index = 0u;
    for(; index < BODY_TOTAL_SIZE; index++) {
        if (!is_body_enabled[index]) {
            break;
        }
    }

    // Compute inertia tensors, inlocal and world spaces
    glm::mat3 inv_inertia_tensor;
    if (new_body.type == SPHERE_COLLIDER) {
        inv_inertia_tensor = glm::mat3(2.0f * new_body.scale.x * new_body.scale.x / 5.0f);
    }

    inv_inertia_tensor = glm::inverse(inv_inertia_tensor);
    glm::mat3 body_rotation = glm::toMat3(new_body.orientation);
    local_inertia_tensors[index] = inv_inertia_tensor * new_body.inv_mass;
    world_inertia_tensors[index] = body_rotation * (inv_inertia_tensor * new_body.inv_mass) * glm::transpose(body_rotation);

    is_body_enabled[index] = true;
    bodies[index] = new_body;

    return index;
}

void sPhysicsWorld::render() {

    glm::mat4 vp_mat;
    camera.get_perspective_viewprojection_matrix(90.0f,
                                                 1000.0f,
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

        model = glm::translate(glm::mat4(1.0f), curr_body.position);
        model = glm::toMat4(curr_body.orientation) * model;
        model = glm::scale(model, curr_body.scale);        

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
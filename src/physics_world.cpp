#include "physics_world.h"

#include "glm/ext/quaternion_geometric.hpp"
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
                .inv_mass = 1.0f / 2.0f,
                .elasticity = 0.5f,
                .position = {0.0f, 4.0f, 0.0f},
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
        ImGui::SliderFloat4("body rot", (float*)&bodies[0].orientation, -10.0f, 10.0f);
        ImGui::SliderFloat3("body pos", (float*)&bodies[0].position, -10.0f, 10.0f);
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
    ImGui::Begin("Physics update");
    // Integrate accelerations into velocity
    for(uint8_t i = 0u; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i] || bodies[i].inv_mass == 0.0f) {
            continue;
        }

        apply_linear_impulse(i, glm::vec3(0.0f, -10.0f, 0.0f) * (1.0f / bodies[i].inv_mass) * delta);
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

                ImGui::Text("Collision point obj %i: %f %f %f and %i: %f %f %f", 
                            i, 
                            collisions_in_frame[collision_count].point_in_body1_world.x,
                            collisions_in_frame[collision_count].point_in_body1_world.y,
                            collisions_in_frame[collision_count].point_in_body1_world.z,
                            j,
                            collisions_in_frame[collision_count].point_in_body2_world.x,
                            collisions_in_frame[collision_count].point_in_body2_world.y,
                            collisions_in_frame[collision_count].point_in_body2_world.z);

                collision_count++;
            }
        }
    }

    // Integrate velocity into position
    for(uint8_t i = 0u; i < BODY_TOTAL_SIZE; i++) {
        if (!is_body_enabled[i] || bodies[i].inv_mass == 0.0f) {
            continue;
        }
        update_body(i, delta);
    }

    ImGui::End();
}


void sPhysicsWorld::update_body(const uint8_t index, const float delta) {
    bodies[index].position += speeds[index].linear_velocity * delta;

    // Angular velocity update
    glm::mat3 body_rotation = glm::toMat3(bodies[index].orientation);
    glm::vec3 center_of_mass = get_center_of_mass_in_world(index);
    glm::vec3 center_of_mass_to_position = bodies[index].position - center_of_mass;

    glm::mat3 orientation = glm::toMat3(bodies[index].orientation);
    glm::mat3 inertia_tensors = orientation * local_inertia_tensors[index] * glm::transpose(orientation);
    glm::vec3 alpha = glm::inverse(inertia_tensors) * (glm::cross(speeds[index].angular_velocity, inertia_tensors * speeds[index].angular_velocity));
    speeds[index].angular_velocity += alpha * delta;

    // Update orientation
    glm::vec3 delta_angle = speeds[index].angular_velocity * delta;
    
    // There must be better way to do this
    const float half_angle_radians = 0.5f * glm::length(delta_angle);
    delta_angle = safe_normalize(delta_angle) * glm::sin(half_angle_radians);
    glm::quat delta_rotation_quaterion = glm::quat(glm::cos(half_angle_radians), delta_angle.x, delta_angle.y, delta_angle.z);

    bodies[index].orientation = glm::normalize(delta_rotation_quaterion * bodies[index].orientation);

    bodies[index].position = center_of_mass + (delta_rotation_quaterion * center_of_mass_to_position);
}   


void sPhysicsWorld::resolve_collision(const sCollisionManifold &manifold) {
    const uint8_t body1 = manifold.body1_index, body2 = manifold.body2_index;

    const float elasticity = bodies[body1].elasticity * bodies[body2].elasticity; 

    const glm::vec3 r1 = manifold.point_in_body1_world - get_center_of_mass_in_world(body1);
    const glm::vec3 r2 = manifold.point_in_body2_world - get_center_of_mass_in_world(body2);

    // Collision impulse
    // Angular factor for the impulse
    const glm::vec3 angular_factor_body1 = glm::cross(get_inverse_inertia_tensor_in_world(body1) * glm::cross(r1, manifold.normal), r1);
    const glm::vec3 angular_factor_body2 = glm::cross(get_inverse_inertia_tensor_in_world(body2) * glm::cross(r2, manifold.normal), r2);
    const float angular_factor = glm::dot(angular_factor_body1 + angular_factor_body2, manifold.normal);

    // Object worlspace velocity
    const glm::vec3 vel_1 = speeds[body1].linear_velocity + glm::cross(speeds[body1].angular_velocity, r1);
    const glm::vec3 vel_2 = speeds[body2].linear_velocity + glm::cross(speeds[body2].angular_velocity, r2);
    const glm::vec3 velocity_delta = vel_1 - vel_2;

    const float inv_mass = bodies[body1].inv_mass + bodies[body2].inv_mass;

    //const float impulse = (1.0f + elasticity) * glm::dot(velocity_delta, manifold.normal) / (bodies[manifold.body1_index].inv_mass + bodies[manifold.body2_index].inv_mass);
    const float impulse = (1.0f + elasticity) * glm::dot(velocity_delta, manifold.normal) / (inv_mass + angular_factor);

    apply_impulse(manifold.body1_index, manifold.normal * impulse * -1.0f, manifold.point_in_body1_world);
    apply_impulse(manifold.body2_index, manifold.normal * impulse, manifold.point_in_body2_world);

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
    glm::mat3 inertia_tensor;
    if (new_body.type == SPHERE_COLLIDER) {
        inertia_tensor = glm::mat3(2.0f * new_body.scale.x * new_body.scale.x / 5.0f);

        local_center_of_mass[index] = glm::vec3(0.0f);
    }

    glm::mat3 body_rotation = glm::toMat3(new_body.orientation);
    // Normal Inertia tensors
    local_inertia_tensors[index] = inertia_tensor;

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

        glm::mat4 translate = glm::translate(glm::mat4(1.0f), curr_body.position);
        glm::mat4 rotation = glm::toMat4(curr_body.orientation);
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), curr_body.scale);

        model = translate * rotation * scale;       

        if (curr_body.type == SPHERE_COLLIDER) {
            sphere_renderer.add(model, curr_body.color);
        }
    }

    for(uint16_t i = 0u; i < collision_count; i++) {
        model = glm::translate(glm::mat4(1.0f), collisions_in_frame[i].point_in_body1_world);
        model = glm::scale(model, glm::vec3(0.05f, 0.05f, 0.05f));
        sphere_renderer.add(model, {1.0f, 0.0f, 0.0f, 1.0f});

        model = glm::translate(glm::mat4(1.0f), collisions_in_frame[i].point_in_body2_world);
        model = glm::scale(model, glm::vec3(0.05f, 0.05f, 0.05f));
        sphere_renderer.add(model, {1.0f, 1.0f, 0.0f, 1.0f});
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
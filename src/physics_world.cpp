#include "physics_world.h"

#include <imgui.h>
#include <glm/glm.hpp>

glm::vec3 rotate_arround(const glm::vec3  &pos, 
                         const glm::vec3  &center, 
                         const float angle);

void sPhysicsWorld::init() {
    sphere_renderer.create_from_file("resources/sphere.obj");

    camera.look_at({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
}
void sPhysicsWorld::update(const float delta) {
    // Instances to render
    {
        sphere_renderer.add(glm::mat4(1.0f), {1.0f, 0.0f, 0.0f, 1.0f});
    }

    // ImGUI & camera controls
    {
        ImGui::Begin("Camera control");

        bool camera_moved = false;
        camera_moved |= ImGui::SliderFloat3("Camera look at", (float*)&camera.center, -5.0f, 5.0f);
        camera_moved |= ImGui::SliderFloat("Camera rotation", &camera_angle, 0.0f, 3.141516f);
        
        camera.position = rotate_arround(camera.position, camera.center, camera_angle);

        if (camera_moved) {
            camera.look_at(camera.center, camera.position);
        }
        ImGui::End();
    }
    
}

void sPhysicsWorld::render() {

    glm::mat4 vp_mat;
    camera.get_perspective_viewprojection_matrix(90.0f,
                                                 10.0f,
                                                 0.1f,
                                                 (float) width / height,
                                                 &vp_mat);

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
  float s = sin(angle), c = cos(angle);

  float nx = pos.x * c - pos.z * s;
  float nz = pos.x * s + pos.z * c;

  return glm::vec3(nx + center.x, pos.y, nz + center.z);
}
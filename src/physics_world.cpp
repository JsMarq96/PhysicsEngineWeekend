#include "physics_world.h"

#include <imgui.h>
#include <glm/glm.hpp>

void sPhysicsWorld::init() {
    sphere_renderer.create_from_file("resources/sphere.obj");

    camera.look_at({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
}
void sPhysicsWorld::update(const float delta) {
    sphere_renderer.add(glm::mat4(1.0f), {1.0f, 0.0f, 0.0f, 1.0f});

    ImGui::Begin("Camera control");
    if (ImGui::SliderFloat3("Camera look at", (float*)&camera.center, -5.0f, 5.0f)) {
        camera.look_at(camera.center, camera.position);
    }
    ImGui::End();
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
#pragma once

#include "render/mesh.h"
#include "render/camera.h"
#include "utils.h"
#include <cstdint>

struct sPhysicsWorld {
    uint32_t width = 0u;
    uint32_t height = 0u;

    Renderer::sCamera    camera;
    Renderer::sMeshRenderer sphere_renderer;
    
    void init();
    void update(const float delta);
    void render();
    void clean();
    void window_resized(const float height, const float width);
};
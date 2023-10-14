#pragma once
#include <cstdint>
#include <glm/glm.hpp>

#define OBJECT_COUNT 100u

namespace Renderer {


    struct sMeshVertex {
        glm::vec3       vertex = {0.0f, 0.0f, 0.0f};
        glm::vec3       normal = {0.0f, 0.0f, 0.0f};
        glm::vec2       uv = {0.0f, 0.0f};
    };


    struct sMeshRenderer {
        uint32_t        VAO = 0u;
        uint32_t        VBO = 0u;

        uint16_t        primitive_count = 0u;

        uint32_t        gl_shader = 0u;

        glm::mat4       models[OBJECT_COUNT];
        glm::vec4       colors[OBJECT_COUNT];
        uint16_t        render_count = 0u;

        void create_from_file(const char* obj_file);
        void render(const glm::mat4 &viewproj_mat) const;
        void clean();
        void add(const glm::mat4 &model, const glm::vec4 &color);
    };
}
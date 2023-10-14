#include "mesh.h"

#include <cstdint>
#include <tiny_obj_loader.h>
#include <GL/gl3w.h>

#include "../utils.h"
#include "gl_shaders.h"

void Renderer::sMeshRenderer::create_from_file(const char* obj_file) {
    // Using tinyObjLoader HOW DO YOU RELEASE THIS?????
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./";

    tinyobj::ObjReader reader;

    assert_msg(reader.ParseFromFile(obj_file, reader_config), 
                "Not found the obj file");

    const tinyobj::shape_t &shape = reader.GetShapes()[0];
    const tinyobj::attrib_t &attrib = reader.GetAttrib();

    // Only support a single object AND triangular
    sMeshVertex *raw_mesh = (sMeshVertex*) malloc(sizeof(sMeshVertex) * 3u * shape.mesh.num_face_vertices.size());
    uint32_t raw_mesh_size = 0u;
    // First get mesh

    uint32_t index_offset = 0u;
    for(uint32_t face_index = 0u; face_index < shape.mesh.num_face_vertices.size(); face_index++) {
        const uint32_t face_vertex_count = shape.mesh.num_face_vertices[face_index];

        assert_msg(face_vertex_count == 3u, "Non triangulated mesh");

        // access to vertex
        for (uint32_t v = 0; v < face_vertex_count; v++) {
            tinyobj::index_t idx = shape.mesh.indices[index_offset + v];

            glm::vec3 vertex = {0.0f, 0.0f, 0.0f}, normal = {0.0f, 0.0f, 0.0f};
            glm::vec2 uv = {0.0f, 0.0f};

            // Load vertex
            {
                vertex.x = attrib.vertices[3u * uint32_t(idx.vertex_index) + 0u];
                vertex.y = attrib.vertices[3u * uint32_t(idx.vertex_index) + 1u];
                vertex.z = attrib.vertices[3u * uint32_t(idx.vertex_index) + 2u];
            }

            // Load normal, if any
            if (idx.normal_index >= 0) {
                normal.x = attrib.normals[3u * uint32_t(idx.normal_index) + 0u];
                normal.y = attrib.normals[3u * uint32_t(idx.normal_index) + 1u];
                normal.z = attrib.normals[3u * uint32_t(idx.normal_index) + 2u];
            }

            // Load UVs, if any
            if (idx.texcoord_index >= 0) {
                uv.x = attrib.texcoords[2u * uint32_t(idx.texcoord_index) + 0u];
                uv.y = attrib.texcoords[2u * uint32_t(idx.texcoord_index) + 1u];
            }

            raw_mesh[raw_mesh_size].vertex = vertex;
            raw_mesh[raw_mesh_size].normal = normal;
            raw_mesh[raw_mesh_size].uv = uv;
            raw_mesh_size++;
        }
        
        index_offset += 3u;
        primitive_count = shape.mesh.num_face_vertices.size();
    }

    // Create OpenGL Buffers
    {
        glGenBuffers(1u, &VBO);
        glGenVertexArrays(1u, &VAO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        // Attribute config
        glEnableVertexAttribArray(0u);
        glVertexAttribPointer(0u, 3u, GL_FLOAT, GL_FALSE, sizeof(sMeshVertex), (void*)0);
        glEnableVertexAttribArray(1u);
        glVertexAttribPointer(1u, 3u, GL_FLOAT, GL_FALSE, sizeof(sMeshVertex), (void*) (sizeof(float) * 3u));
        glEnableVertexAttribArray(2u);
        glVertexAttribPointer(1u, 2u, GL_FLOAT, GL_FALSE, sizeof(sMeshVertex), (void*) (sizeof(float) * 5u));

        glBufferData(GL_ARRAY_BUFFER, primitive_count * 3u * sizeof(sMeshVertex), raw_mesh, GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    // Create shaders
    {
        uint32_t vertex_shader, fragment_shader;
        int32_t   compile_success = 0;

        vertex_shader = glCreateShader(GL_VERTEX_SHADER);

        const GLchar* in_vertex_shader = borring_vertex_shader;
        glShaderSource(vertex_shader, 1, &in_vertex_shader, nullptr);
        glCompileShader(vertex_shader);
        glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &compile_success);

        assert_msg(compile_success, "Failed compile of vertex shader");

        fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

        const GLchar* in_frag_shader = borring_frag_shader;
        glShaderSource(fragment_shader, 1, &in_frag_shader, nullptr);
        glCompileShader(fragment_shader);
        glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &compile_success);
    
        assert_msg(compile_success, "Failed compile of fragment shader");

        gl_shader = glCreateProgram();
        glAttachShader(gl_shader, vertex_shader);
        glAttachShader(gl_shader, fragment_shader);
        glLinkProgram(gl_shader);
        glGetProgramiv(gl_shader, GL_LINK_STATUS, &compile_success);

        assert_msg(compile_success, "Failed linking of shader");

        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
    }
    
    free(raw_mesh);
}

void Renderer::sMeshRenderer::render(const glm::mat4 &viewproj_mat) const {
    // Always wireframe mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBindVertexArray(VAO);

    glUseProgram(gl_shader);

    glUniformMatrix4fv(glGetUniformLocation(gl_shader, "u_viewproj"), 1u, false, (float*) &viewproj_mat);

    for(uint32_t i = 0u; i < render_count; i++) {
        glUniformMatrix4fv(glGetUniformLocation(gl_shader, "u_model"), 1u, false, (float*) &models[i]);
        glUniform4fv(glGetUniformLocation(gl_shader, "u_color"), 1u, (float*) &colors[i]);
        glDrawArrays(GL_TRIANGLES, 0, primitive_count);
    }

    glBindVertexArray(0);
}

void Renderer::sMeshRenderer::clean() {
    glDeleteBuffers(1u, &VBO);
    glDeleteVertexArrays(1u, &VAO);
    glDeleteProgram(gl_shader);
}

void Renderer::sMeshRenderer::add(const glm::mat4 &model, 
                                  const glm::vec4 &color) {
    models[render_count] = model;
    colors[render_count] = color;
    render_count++;
}
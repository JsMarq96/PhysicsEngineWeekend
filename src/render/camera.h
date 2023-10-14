#pragma once

#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

/**
 * Camera functions
 * TODO: Implement zoom as a view matrix
 * */

#define to_radians(FOV) (FOV * 0.0174533f)

namespace Renderer {

    struct sCamera {
        glm::vec3     position = {0.0f, 0.0f, 0.0f};

        glm::vec3    u = {};
        glm::vec3    s = {};
        glm::vec3    f = {};

        glm::mat4 view_mat = {};

        void get_perspective_projection_matrix(const float FOV,
                                            const float far_plane,
                                            const float near_plane,
                                            const float aspect_ratio,
                                            glm::mat4  *result) const {
            float tan_half_FOV = tan(to_radians(FOV) / 2.0f);

            *result = glm::perspective(FOV,
                                        aspect_ratio,
                                        near_plane,
                                        far_plane);
        }

        void look_at(const glm::vec3 &center) {
            f = glm::normalize(glm::vec3{center.x - position.x, center.y - position.y, center.z - position.z});
            s = glm::normalize(glm::cross(f, glm::vec3{0.f, 1.0f, 0.0f}));
            u = glm::cross(s, f);

            view_mat = glm::lookAt(position,
                                    center,
                                    u);
        }

        void look_at(const glm::vec3 &center, const glm::vec3 &from) {
            position = from;
            f = glm::normalize(glm::vec3{center.x - position.x, center.y - position.y, center.z - position.z});
            s = glm::normalize(glm::cross(f, glm::vec3{0.f, 1.0f, 0.0f}));
            u = glm::cross(s, f);

            view_mat = glm::lookAt(position,
                                    center,
                                    u);
        }

        void get_perspective_viewprojection_matrix(const float FOV,
                                                const float far_plane,
                                                const float near_plane,
                                                const float aspect_ratio,
                                                glm::mat4  *result) const {
            glm::mat4 persp = {};
            get_perspective_projection_matrix(FOV, far_plane, near_plane, aspect_ratio, &persp);

            *result = persp * view_mat;
        }
    };
}
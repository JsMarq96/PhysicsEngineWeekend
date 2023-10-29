#pragma once
#include "glm/geometric.hpp"
#include <iostream>
#include <cassert>
#include <glm/glm.hpp>

inline void assert_msg(const bool res, const char* msg) {
    if (!res) {
        std::cout << "ASSERT: " << msg << std::endl;
        assert(false);
    }
}

inline glm::vec3 safe_normalize(const glm::vec3 &v) {
    const float mag = 1.0f / glm::length(mag);

    if (mag * 0.0f == mag * 0.0f) {
        return v * mag;
    }

    return v;
}
#pragma once
#include <iostream>
#include <cassert>

inline void assert_msg(const bool res, const char* msg) {
    if (!res) {
        std::cout << "ASSERT: " << msg << std::endl;
        assert(false);
    }
}
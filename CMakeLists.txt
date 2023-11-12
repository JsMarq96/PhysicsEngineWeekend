cmake_minimum_required(VERSION 3.21)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_PARALLEL_LEVEL 6)

project(phys_weekend)

file(GLOB H_SOURCES "src/*.h")
file(GLOB CPP_SOURCES "src/*.cpp")
file(GLOB C_SOURCES "src/*.c")
file(GLOB SUBFOLDER_H_SOURCES "src/*/*.h")
file(GLOB SUBFOLDER_CPP_SOURCES "src/*/*.cpp")
file(GLOB SUBFOLDER_C_SOURCES "src/*/*.c")

# ImGUI
include_directories(libraries/imgui/)
include_directories(libraries/imgui/backends/)
set(IMGUI_IMPL_SRC libraries/imgui/backends/imgui_impl_glfw.cpp 
                   libraries/imgui/backends/imgui_impl_opengl3.cpp)
file(GLOB IMGUI_SRC "libraries/imgui/*.cpp")

add_executable(phys_weekend ${IMGUI_IMPL_SRC} ${IMGUI_SRC} ${C_SOURCES} ${CPP_SOURCES} ${H_SOURCES} ${SUBFOLDER_H_SOURCES} ${SUBFOLDER_C_SOURCES} ${SUBFOLDER_CPP_SOURCES})

target_compile_options(phys_weekend PRIVATE -pthread)
set_target_properties(phys_weekend PROPERTIES OUTPUT_NAME "phys_weekend")

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(libraries/glfw)

add_subdirectory(libraries/gl3w)

add_subdirectory(libraries/glm)

add_subdirectory(libraries/tinyobjloader)

target_link_libraries(phys_weekend glm glfw gl3w tinyobjloader)

add_custom_command(TARGET phys_weekend POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:phys_weekend>/resources)
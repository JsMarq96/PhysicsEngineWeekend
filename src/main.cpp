#include <stdio.h>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h> 
#include <glm/glm.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include "render/mesh.h"
#include "render/camera.h"

#define WIN_WIDTH	640
#define WIN_HEIGHT	480
#define WIN_NAME	"Test Physics perron"

typedef struct sAppState {
    GLFWwindow *window;
    bool close_window;
} sAppState;

sAppState app_state;

int main() {
    if (!glfwInit()) {
        printf("Error initializing glfw\n");
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    app_state.window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, WIN_NAME, NULL, NULL);
    
    glfwMakeContextCurrent(app_state.window);
    glfwSwapInterval(1);

    if (!app_state.window) {
        printf("Error creating window\n");
        return -1;
    }

    if (gl3wInit()) {
        printf("Error intializing GL context\n");
        return -1;
    }

    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    // Platform IMGUI
    ImGui_ImplGlfw_InitForOpenGL(app_state.window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    ImGui::StyleColorsDark();

    app_state.close_window = false;

    int32_t width, height;

    double prev_frame_time = 0.0;


    glfwGetFramebufferSize(app_state.window, &width, &height);

    Renderer::sCamera camera;

    camera.look_at({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});

    Renderer::sMeshRenderer sphere_render;
    sphere_render.create_from_file("resources/sphere.obj");

    while(!glfwWindowShouldClose(app_state.window) && !app_state.close_window) {
        glfwMakeContextCurrent(app_state.window);
        glfwPollEvents();
        glfwGetFramebufferSize(app_state.window, &width, &height);
        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);

        double curr_time = glfwGetTime();
        double delta_time = curr_time - prev_frame_time;
        prev_frame_time = curr_time;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Update here
        glm::mat4 vp_mat;
        camera.get_perspective_viewprojection_matrix(90.0f,
                                                     10.0f,
                                                     0.01f,
                                                     (float) width / height,
                                                     &vp_mat);

        // Frame render here
        sphere_render.add(glm::mat4(1.0f), {1.0, 0.0, 0.0, 1.0f});

        sphere_render.render(vp_mat);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(app_state.window);

        // Close window and stop loop
        app_state.close_window = glfwGetKey(app_state.window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
    }

    sphere_render.clean();
    
    return 0;
}
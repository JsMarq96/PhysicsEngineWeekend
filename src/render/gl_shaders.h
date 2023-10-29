#pragma once

/**
* Raw OpenGL shaders
*/

 const char borring_vertex_shader[] = "#version 410 \n"
                                          "layout(location = 0) in vec3 a_v_pos;\n"
                                          "layout(location = 1) in vec3 a_v_normal;\n"
                                          "layout(location = 2) in vec2 a_v_uv;\n"
                                          "out vec3 v_color;\n"
                                          "uniform mat4 u_model;\n"
                                          "uniform mat4 u_viewproj;\n"
                                          "void main() {\n"
                                          " v_color = a_v_normal;\n"
                                          "	gl_Position = u_viewproj * u_model * vec4(a_v_pos, 1.0);\n"
                                          "}\n";

 const char borring_frag_shader[] = "#version 410\n"
                                          "layout(location = 0) out vec4 out_color;\n"
                                          "uniform vec4 u_color; \n"
                                          "in vec3 v_color;\n"
                                          "void main() {\n"
                                          "	out_color = vec4(v_color, 1.0);\n"
                                          "}\n";
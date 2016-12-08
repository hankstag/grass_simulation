// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"
#include "Grass.h"
#include "Ground.h"
#include "util.h"
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
#include <iostream>
// Linear Algebra Library
#include <Eigen/Core>
#include <Eigen/Dense>

// Timer
#include <chrono>
using namespace std;
using namespace Eigen;

#define WIRE 1
#define FLAT 2
#define PHONG 3
#define BEZIER 4

#define INIT 5
#define ANIMATION 6
#define ADDING 7

#define ORTH 8
#define PREP 9

Eigen::Vector3f light;
Eigen::Vector3f camera;
Eigen::Vector3f gaze; // direction of camera
Eigen::Vector3f up_direction; // the direction pointing up
int state=INIT;
int mode=ORTH;

vector<VertexBufferObject> objs;
vector<VertexBufferObject> objs_color;
vector<VertexBufferObject> objs_normals;
vector<int> obj_size;

MatrixXf Mcam(4,4),Morth(4,4),Perspective(4,4),view(4,4);

vector<Matrix4f> translate_mat;
vector<Matrix4f> to_center_mat;
vector<Matrix4f> scale_mat;
vector<Matrix4f> rotate_mat;

std::chrono::high_resolution_clock::time_point t_start;
std::chrono::high_resolution_clock::time_point t_now;

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    // Get the position of the mouse in the window
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the size of the window
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Convert screen position to world coordinates
    double xworld = ((xpos/double(width))*2)-1;
    double yworld = (((height-1-ypos)/double(height))*2)-1; // NOTE: y axis is flipped in glfw
    Eigen::Vector4f ray_origin;
    ray_origin<<xworld,yworld,0,1;
    ray_origin=view.inverse()*ray_origin;
    //view * Morth * Mcam * transform *
    // Update the position of the first vertex if the left button is pressed
    MatrixXf C;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
    }
}

void move_camera(Eigen::Vector3f& camera, int direction){
    Vector3f w,u,v;
    w=-gaze.normalized();
    u=up_direction.cross(w).normalized();
    v=w.cross(u);
    switch (direction) {
        case 1:camera+=v;camera.normalize();camera=camera*2;break;
        case 2:camera-=v;camera.normalize();camera=camera*2;break;
        case 3:camera+=u;camera.normalize();camera=camera*2;break;
        case 4:camera-=u;camera.normalize();camera=camera*2;break;
        default:break;
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    if(action != GLFW_PRESS) return;
    Matrix4f rotate(4,4);
    rotate<<0.9848f,0.1736f,0,0,
    -0.1736f,0.9848f,0,0,
    0,0,1,0,
    0,0,0,1;
    switch (key)
    {
        case GLFW_KEY_R:
            rotate_mat[0] *= rotate;
            break;
        case GLFW_KEY_U:
            // move center to camera
            scale_mat[0](0,0)*=1.25;
            scale_mat[0](1,1)*=1.25;
            scale_mat[0](2,2)*=1.25;
            break;
        case GLFW_KEY_I:
            scale_mat[0](0,0)*=0.8;
            scale_mat[0](1,1)*=0.8;
            scale_mat[0](2,2)*=0.8;
           break;
        case GLFW_KEY_W:
            move_camera(camera,1);
            gaze<<-camera;
            break;
        case GLFW_KEY_S:
            move_camera(camera,2);
            gaze<<-camera;
            break;
        case GLFW_KEY_A:
            move_camera(camera,3);
            gaze<<-camera;
            break;
        case GLFW_KEY_D:
            move_camera(camera,4);
            gaze<<-camera;
            break;
        case GLFW_KEY_4:
            // toggle mode
            if(mode == ORTH)
                mode = PREP;
            else
                mode = ORTH;
        default:
            break;
    }

}

int main(void)
{
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex
    // attributes are stored in a Vertex Buffer Object (or VBO). This means that
    // the VAO is not the actual object storing the vertex data,
    // but the descriptor of the vertex data.
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    camera<<2,2,2;
    camera.normalize();
    camera*=2;
    gaze<<-camera;
    light<<-10,-10,-10;
    up_direction<<0,0,-1;
    init_ground(light,objs,objs_color,objs_normals,obj_size,translate_mat,to_center_mat,scale_mat,rotate_mat);

    Grass grass;
    grass.draw(objs,objs_color,objs_normals,light,obj_size,translate_mat,to_center_mat,scale_mat,rotate_mat);
    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;"
                    "in vec3 normal;"
                    "uniform vec3 light;"
                    "uniform mat4 view;"
                    "uniform mat4 transform;"
                    "uniform mat4 Mcam;"
                    "uniform mat4 Morth;"
                    "uniform mat4 Perspective;"
                    "in vec3 color;"
                    "out vec3 f_color;"
                    "void main()"
                    "{"
                    "    vec4 intermediate =  Perspective * Mcam * transform * vec4(position, 1.0);"
                    "    intermediate /= intermediate.w;"
                    "    gl_Position = view * Morth * intermediate;"
                    //"    gl_Position = transform*vec4(position, 1.0);"
                    "    vec4 N = vec4(normal,0.0);"
                    "    vec4 L;"
                    "    vec4 V;"
                    "    mat4 T=inverse(transpose(view*transform));"
                    "    V = vec4(position,1.0);"
                    "    L = vec4(light,1.0);"
                    "    f_color = max(dot(normalize(L-transform*V),normalize(T*N)),0.0f)*color;"
                    //"   f_color=color;"
                    "}";

    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec3 f_color;"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(f_color, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    // // The vertex shader wants the position of the vertices as an input.
    // // The following line connects the VBO we defined above with the position "slot"
    // // in the vertex shader
    // program.bindVertexAttribArray("position",VBO);

    // Save the current time --- it will be used to dynamically change the triangle color
    //auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Loop until the user closes the window

    while (!glfwWindowShouldClose(window))
    {
        // Bind your VAO (not necessary if you have only one)
        VAO.bind();
        // The vertex shader wants the position of the vertices as an input.
        // The following line connects the VBO we defined above with the position "slot"
        // in the vertex shader
        // for every vbo, do the followiing things
        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        for(int i=0;i<objs.size();i++){
            program.bindVertexAttribArray("position",objs[i]);
            program.bindVertexAttribArray("color",objs_color[i]);
            program.bindVertexAttribArray("normal",objs_normals[i]);
            // Bind your program
            program.bind();

            // Get size of the window
            int width, height;
            glfwGetWindowSize(window, &width, &height);
            float aspect_ratio = float(height)/float(width); // corresponds to the necessary width scaling
            view <<
            aspect_ratio,0, 0, 0,
            0,           1, 0, 0,
            0,           0, 1, 0,
            0,           0, 0, 1;
            // MatrixXf neg_z(4,4);
            // neg_z<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
            glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());
            glUniform3fv(program.uniform("light"), 1, light.data());
            //glUniformMatrix4fv(program.uniform("neg_z"), 1, GL_FALSE, neg_z.data());
            MatrixXf transform =  translate_mat[i] * scale_mat[i] * rotate_mat[i] * to_center_mat[i];
            Vector3f w,u,v;
            w=-gaze.normalized();
            u=up_direction.cross(w).normalized();
            v=w.cross(u);

            MatrixXf M1(4,4),M2(4,4);
            M1<<u(0),u(1),u(2),0,v(0),v(1),v(2),0,w(0),w(1),w(2),0,0,0,0,1;
            M2<<1,0,0,-camera(0),0,1,0,-camera(1),0,0,1,-camera(2),0,0,0,1;
            Mcam=M1*M2;

            float l=-1.0f,b=-1.0f,n=-1.0f,r=1.0f,t=1.0f,f=-10.0f;
            Morth<<2.0f/(r-l),0,0,-(r+l)/(r-l),
                   0,2.0f/(t-b),0,-(t+b)/(t-b),
                   0,0,2.0f/(n-f),-(n+f)/(n-f),
                   0,0,0,1;
            if(mode==ORTH)
                Perspective<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
            else
                Perspective<<n,0,0,0,
                         0,n,0,0,
                         0,0,n+f,-f*n,
                         0,0,1,0;
            glUniformMatrix4fv(program.uniform("transform"), 1, GL_FALSE, transform.data());
            glUniformMatrix4fv(program.uniform("Mcam"), 1, GL_FALSE, Mcam.data());
            glUniformMatrix4fv(program.uniform("Perspective"), 1, GL_FALSE, Perspective.data());
            glUniformMatrix4fv(program.uniform("Morth"), 1, GL_FALSE, Morth.data());

            if(i==0){
                int triangle_number = obj_size[i]/3;
                // Draw a triangle
                glDrawArrays(GL_TRIANGLES, 0, triangle_number*3);
            }else{
                glDrawArrays(GL_LINES, 0, 100);
            }

        }
        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();
    VAO.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}

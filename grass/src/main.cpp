// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"
#include "Grass.h"
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
#define WIRE 1
#define FLAT 2
#define PHONG 3
#define BEZIER 4

#define INIT 5
#define ANIMATION 6
#define ADDING 7

#define ORTH 8
#define PREP 9
// VertexBufferObject wrapper
VertexBufferObject VBO;
VertexBufferObject VBO_C;

// Contains the view transformation
Eigen::Matrix4f view(4,4);

// Contains the vertex positions
Eigen::MatrixXf V(2,3);
using namespace std;
using namespace Eigen;
Eigen::Vector3f light;
Eigen::Vector3f camera;
Matrix4f M;
vector<MatrixXf> moves;
Eigen::Vector3f gaze; // direction of camera
Eigen::Vector3f up_direction; // the direction pointing up
int option=FLAT;
int obj_select = 0; // default pick no.0;
int state=INIT;
int mode=ORTH;

vector<VertexBufferObject> objs;
vector<VertexBufferObject> objs_color;
vector<VertexBufferObject> objs_normals;
vector<VertexBufferObject> objs_barycenter;
vector<int> obj_size;
vector<int> opts; // the option for each objs
vector<vector<Vector4f>> ctr_pts_vec; // vector of control points of each curve
vector<Vector4f> ctr_pts; // control points
vector<MatrixXf> curves; // stores the sample
vector<Vector4f> v_old;
MatrixXf sample_pts(4,100); // sample_pts of curve
vector<Vector3f> b_center; // barycenters
unordered_map<int,int> m_to_c;

vector<MatrixXf> NFS;
vector<MatrixXf> NVS;
vector<MatrixXi> FS;
vector<MatrixXf> VS; // vertices for all object

MatrixXf Mcam(4,4),Morth(4,4),Mvp(4,4),Perspective(4,4);

vector<Matrix4f> translate_mat;
vector<Matrix4f> to_center_mat;
vector<Matrix4f> scale_mat;
vector<Matrix4f> rotate_mat;
vector<vector<vector<int>>> adj_map; // each vertices and its corresponding faces

std::chrono::high_resolution_clock::time_point t_start;
std::chrono::high_resolution_clock::time_point t_now;

int getIntersectTriangle(
    const Eigen::MatrixXf V,
    const Eigen::Vector3f ray_origin
){
    for(int k=0;k<V.cols()-2;k+=3){
        Eigen::MatrixXf A(3,3);
        Eigen::Vector3f b(3);
        Eigen::Vector3f ray_direction(3);
        ray_direction<<0.0f,0.0f,-1.0f;
        Eigen::Vector3f ab(3);
        ab<<V.col(k)-V.col(k+1);
        Eigen::Vector3f ac(3);
        ac<<V.col(k)-V.col(k+2);
        A<<ab,ac,ray_direction;
        b<<V.col(k)-ray_origin;
        //std::cout<<b<<std::endl;
        Eigen::Vector3f s = A.householderQr().solve(b);
        if(s(1)>=0&&s(1)<=1&&s(0)>=0&&s(0)<=1&&s(0)+s(1)<=1){
            // valid intersection
            cout<<k<<endl;
            return k;
        }
    }
    return -1;
}

void bezier_locate(float t, vector<Vector4f>& ctr_pts, vector<Vector4f>& pos){
    //cout<<"get locate at para t = "<<t<<endl;
    int n = ctr_pts.size();
    //cout<<n<<endl;
    if(n==1){
        pos.resize(1);
        pos[0] = ctr_pts[0];
        return;
    }
    vector<Vector4f> tmp;
    Vector4f s;
    tmp.clear();
    for(int i=1;i<n;i++){
        s = t*ctr_pts[i]+(1-t)*ctr_pts[i-1];
        tmp.push_back(s);
    }
    bezier_locate(t,tmp,pos);
    return;
}

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
    int tmp = obj_select;
    MatrixXf C;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
        // check intersection first
        for(int i=0;i<VS.size();i++){
            MatrixXf transform =  translate_mat[i] * scale_mat[i] * rotate_mat[i] * to_center_mat[i];
            MatrixXf vs4(4,VS[i].cols());
            VectorXf ones(VS[i].cols());
            for(int j=0;j<VS[i].cols();j++)ones(j)=1;
            vs4<<VS[i],ones.transpose();
            vs4=Morth*Perspective*Mcam*transform*vs4;
            for(int i=0;i<vs4.cols();i++)
                vs4.col(i)/=vs4(3,i);
            //cout<<Morth*Mcam*transform*to_center_mat[i].col(3)<<endl;
            //cout<<ray_origin.block(0,0,3,1)<<endl;
            int id = getIntersectTriangle(vs4.block(0,0,3,VS[i].cols()),ray_origin.block(0,0,3,1));
            if(id!=-1){
                // restore old color
                C.resize(3,VS[obj_select].cols());
                for(int i=0;i<VS[obj_select].cols();i++){
                    C(0,i)=0.18f;
                    C(1,i)=0.34f;
                    C(2,i)=0.47f;
                }
                objs_color[obj_select].update(C);
                obj_select=i;
                break;
            }
        }
    }
    C.resize(3,VS[obj_select].cols());
    for(int i=0;i<VS[obj_select].cols();i++){
        C(0,i)=1.0f;
        C(1,i)=0.0f;
        C(2,i)=0.0f;
    }
    objs_color[obj_select].update(C);
    //cout<<"select "<<obj_select<<endl;
    // // Upload the change to the GPU
    // VBO.update(V.block(0,0,2,V.cols()));
    // VBO_C.update(C);
}

void update_norm(MatrixXi& F, MatrixXf& N_F, MatrixXf& N_V, MatrixXf& N_P, int option){
    if(option == FLAT){
        for(int i=0;i<N_P.cols()-2;i+=3){
            N_P.col(i)<<N_F.row(i/3).transpose();
            N_P.col(i+1)<<N_F.row(i/3).transpose();
            N_P.col(i+2)<<N_F.row(i/3).transpose();
        }
    }else{
        for(int i=0;i<N_P.cols();i++){
            N_P.col(i)<<N_V.col(F(i));
        }
    }
}

void calculate_norm(MatrixXf& V, MatrixXi& F, vector<vector<int>>& adj, MatrixXf& barycenter, MatrixXf& N_P, int option){
    // V: #P 3
    // F: #elements 1
    Vector3f normal;
    Vector3f ab;
    Vector3f ac;
    MatrixXf N_F(F.rows()/3,3); // normal of every face
    barycenter.resize(3,F.rows()); //barycenter of every face
    MatrixXf N_V(3,V.rows()); // normal of every vertex
    N_P.resize(3,F.rows());
    for(int i=0;i<F.rows()-2;i+=3){
        // F(i) is the vertex
        int a = F(i);
        int b = F(i+1);
        int c = F(i+2);
        //std::cout<<a<<","<<b<<endl;
        ab=V.row(a)-V.row(b);
        ac=V.row(a)-V.row(c);
        barycenter.col(i)  <<((V.row(a)+V.row(b)+V.row(c))/3).transpose();
        barycenter.col(i+1)<<((V.row(a)+V.row(b)+V.row(c))/3).transpose();
        barycenter.col(i+2)<<((V.row(a)+V.row(b)+V.row(c))/3).transpose();
        // std::cout<<ab<<endl;
        Vector3f l=(light-barycenter.col(i)).normalized();
        normal = (ab).cross(ac).normalized();
        N_F.row(i/3)<<normal.transpose();
        // for(int k=0;k<3;k++){
        //     for(int j=0;j<3;j++)
        //         C(j,i+k) = C(j,i+k)* max(.0f,normal.dot(l))+0.1;
        // }
    }
    for(int i=0;i<adj.size();i++){
        //for every vertex, compute the averge normal
        Vector3f sum=Vector3f::Zero();
        for(int j=0;j<adj[i].size();j++)
            sum+=N_F.row(adj[i][j]);
        N_V.col(i)<<sum/adj[i].size();
    }
    NFS.push_back(N_F);
    NVS.push_back(N_V);
    if(option == FLAT){
        for(int i=0;i<F.rows()-2;i+=3){
            N_P.col(i)<<N_F.row(i/3).transpose();
            N_P.col(i+1)<<N_F.row(i/3).transpose();
            N_P.col(i+2)<<N_F.row(i/3).transpose();
        }
    }else{
        for(int i=0;i<F.rows();i++){
            N_P.col(i)<<N_V.col(F(i));
        }
    }
    // for(int i=0;i<F.rows()-2;i+=3){
    //     for(int k=0;k<3;k++){
    //         Vector3f l=(light-V.row(F(i+k)).transpose()).normalized();
    //         for(int j=0;j<3;j++)
    //             C(j,i+k) = C(j,i+k)* max(.0f,N_P.col(i+k).dot(l))+0.1;
    //     }
    // }
}
void create_adj(MatrixXf& box, MatrixXi& F,vector<vector<int>>& adj){
    vector<int> emt={};
    for(int i=0;i<box.rows();i++){
        adj.push_back(emt);
    }
    for(int i=0;i<F.rows();i++){
        adj[F(i)].push_back(i/3);
    }
}

void initialize_obj(MatrixXf& box, MatrixXi& F,MatrixXf& V,MatrixXf& C,MatrixXf& N_V, MatrixXf& barycenter, Matrix4f& scale, Matrix4f& translate, Matrix4f& rotate, Matrix4f& to_center, vector<vector<int>>& adj, int option){

    Vector3f center=Vector3f::Zero();
    translate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    V.resize(3,F.rows());
    C.resize(3,F.rows());
    for(int i=0;i<F.rows();i++){
        for(int j=0;j<3;j++)
            V(j,i) = box(F(i),j);
        C(0,i)=0.6f;C(1,i)=0.4f;C(2,i)=0.2f;
    }
    VS.push_back(V);
    create_adj(box,F,adj);
    calculate_norm(box, F, adj, barycenter,N_V, option);
    for(int i=0;i<3;i++){
        for(int j=0;j<box.rows();j++){
            center(i)+=box(j,i);
        }
        center(i) = center(i)/box.rows();
    }
    b_center.push_back(center);
    rotate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    to_center<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    M<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    to_center(0,3)=-center(0);
    to_center(1,3)=-center(1);
    to_center(2,3)=-center(2);
    Vector4f x=Vector4f::Zero();
    moves.push_back(M);
    v_old.push_back(x);
    to_center_mat.push_back(to_center);
    rotate_mat.push_back(rotate);
    translate_mat.push_back(translate);
    scale_mat.push_back(scale);
    obj_size.push_back(F.rows());

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

void add_vbo(vector<VertexBufferObject>& objs,VertexBufferObject& vbo,MatrixXf& V){
    vbo.init();
    vbo.update(V);
    objs.push_back(vbo);
}
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    if(action != GLFW_PRESS) return;
    Eigen::MatrixXf box;
    Eigen::MatrixXi F;
    Eigen::MatrixXf C; // color for every vertex
    MatrixXf barycenter;
    Eigen::MatrixXf N_F; // normal at every point
    Eigen::MatrixXf N_V; // normal at every vertex
    Eigen::MatrixXf N_P; // normal at every vertex

    VertexBufferObject new_vbo;
    VertexBufferObject new_vbo_color;
    VertexBufferObject new_vbo_normal;
    VertexBufferObject new_vbo_barycenter;
    Matrix4f scale(4,4);
    Matrix4f translate(4,4);
    Matrix4f rotate(4,4);
    Matrix4f to_center(4,4);
    Vector4f center=Vector4f::Zero(4);
    rotate<<0.9848f,0.1736f,0,0,
    -0.1736f,0.9848f,0,0,
    0,0,1,0,
    0,0,0,1;
    vector<vector<int>> adj;
    Matrix4f p;
    MatrixXf dummy,vv;
    Vector4f a,b,c;
    float t = 0.0f;
    int count = 0;
    switch (key)
    {
        case  GLFW_KEY_1:
            //V.col(0) << -0.5,  0.5;
            box.resize(8,3);
            F.resize(36,1);
            box<<1,-1,-1,1,-1,1,-1,-1,1,-1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1;
            F << 4,0,3,4,3,7,2,6,7,2,7,3,1,5,2,5,6,2,0,4,1,4,5,1,4,7,5,7,6,5,0,1,2,0,2,3;
            // create adjcency list
            scale<<0.6,0,0,0,0,0.6,0,0,0,0,0.03,0,0,0,0,1;
            FS.push_back(F);
            opts.push_back(FLAT);
            initialize_obj(box, F, V, C, N_V, barycenter,scale, translate, rotate, to_center,adj,option);
            add_vbo(objs,new_vbo,V);
            add_vbo(objs_color,new_vbo_color,C);
            add_vbo(objs_normals,new_vbo_normal,N_V);
            add_vbo(objs_barycenter,new_vbo_barycenter,barycenter);
            break;
        case GLFW_KEY_R:
            rotate_mat[obj_select] *= rotate;
            break;
        case GLFW_KEY_U:
            // move center to camera
            p<<1,0,0,0,
               0,1,0,0,
               0,0,1,0,
               0,0,0,1;
            scale_mat[obj_select](0,0)*=1.25;
            scale_mat[obj_select](1,1)*=1.25;
            scale_mat[obj_select](2,2)*=1.25;
            break;
        case GLFW_KEY_I:
            scale_mat[obj_select](0,0)*=0.8;
            scale_mat[obj_select](1,1)*=0.8;
            scale_mat[obj_select](2,2)*=0.8;
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

    // Initialize the VBO with the vertices data
    // A VBO is a data container that lives in the GPU memory
    VBO.init();

    camera<<2,2,2;
    camera.normalize();
    camera*=2;
    gaze<<-camera;
    light<<-10,-10,-10;
    up_direction<<0,0,-1;
    Grass grass;
    grass.draw(objs,objs_color,objs_normals,objs_barycenter,opts);
    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;"
                    "in vec3 normal;"
                    "in vec3 barycenter;"
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

    float t=0;
    Vector4f v,v0,a,b,c;

    M<<1,0,0,0,
          0,1,0,0,
          0,0,1,0,
          0,0,0,1;

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
            program.bindVertexAttribArray("barycenter",objs_barycenter[i]);
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
            // cout<<up_direction<<endl;
            // cout<<endl;
            // cout<<w<<endl;
            // cout<<endl;
            // cout<<u<<endl;
            // cout<<endl;
            // cout<<v<<endl;
            MatrixXf M1(4,4),M2(4,4);
            M1<<u(0),u(1),u(2),0,v(0),v(1),v(2),0,w(0),w(1),w(2),0,0,0,0,1;
            M2<<1,0,0,-camera(0),0,1,0,-camera(1),0,0,1,-camera(2),0,0,0,1;
            Mcam=M1*M2;
            //cout<<Mvp<<endl;
            //cout<<Mcam<<endl;
            //Mcam=Mcam.inverse();
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

            int triangle_number = obj_size[i]/3;
            // Draw a triangle
            glDrawArrays(GL_TRIANGLES, 0, triangle_number*3);

        }
        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}

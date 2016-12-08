#include "ground.h"
void init_ground(VertexBufferObject& grd_vbo, VertexBufferObject& color_vbo){
    Eigen::MatrixXf grd;
    Eigen::MatrixXi face;
    Eigen::MatrixXf color;
    Eigen::MatrixXf V;
    grd.resize(8,3);
    face.resize(36,1);
    //grd<<1,-1,-1,1,-1,1,-1,-1,1,-1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1;
    grd<<0.5,-0.5,-0.5,0.5,-0.5,0.5,-0.5,-0.5,0.5,-0.5,-0.5,-0.5,0.5,0.5,-0.5,0.5,0.5,0.5,-0.5,0.5,0.5,-0.5,0.5,-0.5;
    face << 4,0,3,4,3,7,2,6,7,2,7,3,1,5,2,5,6,2,0,4,1,4,5,1,4,7,5,7,6,5,0,1,2,0,2,3;
    V.resize(3,face.rows());
    color.resize(3,face.rows());
    for(int i=0;i<face.rows();i++){
        for(int j=0;j<3;j++)
            V(j,i) = grd(face(i),j);
        color(0,i)=0.18f;color(1,i)=0.34f;color(2,i)=0.47f;
    }
    grd_vbo.init();
    grd_vbo.update(V);
    color_vbo.init();
    color_vbo.update(color);
    //objs.push_back(vbo);
}

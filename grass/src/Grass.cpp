#include "Grass.h"
#include <iostream>
using namespace std;

Grass::Grass(){
    std::cout<<"init grass"<<std::endl;
    ctrl_pts.resize(3,5);
    ctrl_pts.col(0)<<0,-2,0;
    ctrl_pts.col(1)<<0,0,0;
    ctrl_pts.col(2)<<-1,1,-1;
    ctrl_pts.col(3)<<-2,2,-2;
    ctrl_pts.col(4)<<-3,3,-3;
    cout<<ctrl_pts<<endl;
}

void Grass::draw(
    vector<VertexBufferObject>& objs,
    vector<VertexBufferObject>& objs_color,
    vector<VertexBufferObject>& objs_normals,
    vector<VertexBufferObject>& objs_barycenter,
    vector<int>& opts
){
    cout<<"draw me!"<<endl;
    // pass the grass in as a new vbo
}

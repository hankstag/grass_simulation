#ifndef UTIL_H
#define UTIL_H

#include "Helpers.h"
#include <Eigen/Core>
#include <vector>
// void readOff(std::string filename, MatrixXf& V,MatrixXi& F){
//     ifstream readIn;
//     readIn.open(filename,ios::in);
//     if(!readIn.is_open()){
//         cout<<"Error opening file!"<<endl;
//         return;
//     }
//     std::string line;
//     readIn>>line;//first line
//     int vertices;
//     int faces;
//     int edges;
//     readIn>>vertices;
//     readIn>>faces;
//     readIn>>edges;
//     V.resize(vertices,3);
//     F.resize(faces*3,1);
//     for(int i=0;i<vertices;i++){
//         readIn>>V(i,0);
//         readIn>>V(i,1);
//         readIn>>V(i,2);
//     }
//     for(int j=0;j<faces;j++){
//         int a;
//         readIn>>a;
//         readIn>>F(3*j);
//         readIn>>F(3*j+1);
//         readIn>>F(3*j+2);
//     }
//     readIn.close();
//     cout<<"read file done"<<endl;
// }

void add_vbo(std::vector<VertexBufferObject>& objs,Eigen::MatrixXf& V){
    VertexBufferObject vbo;
    vbo.init();
    vbo.update(V);
    objs.push_back(vbo);
}
#endif

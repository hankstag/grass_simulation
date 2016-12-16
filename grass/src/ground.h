#ifndef GROUND_H
#define GROUND_H

#include <Eigen/Core>
#include "Helpers.h"
#include "util.h"

using namespace Eigen;

void calculate_norm(Vector3f& light, MatrixXf& V, MatrixXi& F, MatrixXf& barycenter, MatrixXf& N_P){
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
    for(int i=0;i<F.rows()-2;i+=3){
        N_P.col(i)<<N_F.row(i/3).transpose();
        N_P.col(i+1)<<N_F.row(i/3).transpose();
        N_P.col(i+2)<<N_F.row(i/3).transpose();
    }

}

void init_ground(
    Vector3f& light,
	vector<VertexBufferObject>& objs,
    vector<VertexBufferObject>& objs_uv,
    vector<VertexBufferObject>& objs_color,
    vector<VertexBufferObject>& objs_normals,
    vector<int>& obj_size,
    vector<Matrix4f>& translate_mat,
    vector<Matrix4f>& to_center_mat,
    vector<Matrix4f>& scale_mat,
    vector<Matrix4f>& rotate_mat
){

    Eigen::MatrixXf box;
	Eigen::MatrixXf UV;
    Eigen::MatrixXi F;
    Eigen::MatrixXf C; // color for every vertex
    // Contains the vertex positions
    Eigen::MatrixXf V(2,3);
    MatrixXf barycenter;
    Eigen::MatrixXf N_F; // normal at every point
    Eigen::MatrixXf N_V; // normal at every vertex
    Eigen::MatrixXf N_P; // normal at every vertex

    Matrix4f scale(4,4);
    Matrix4f translate(4,4);
    Matrix4f rotate(4,4);
    Matrix4f to_center(4,4);
    Vector4f center=Vector4f::Zero(4);
    rotate<<0.9848f,0.1736f,0,0,
    -0.1736f,0.9848f,0,0,
    0,0,1,0,
    0,0,0,1;

    box.resize(8,3);
    F.resize(36,1);
    box<<1,-1,-1,1,-1,1,-1,-1,1,-1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,1,-1;
    F << 4,0,3,4,3,7,2,6,7,2,7,3,1,5,2,5,6,2,0,4,1,4,5,1,4,7,5,7,6,5,0,1,2,0,2,3;
    // create adjcency list
    scale<<0.6,0,0,0,0,0.6,0,0,0,0,0.02,0,0,0,0,1;
    translate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    V.resize(3,F.rows());
    C.resize(3,F.rows());
    for(int i=0;i<F.rows();i++){
        for(int j=0;j<3;j++)
            V(j,i) = box(F(i),j);
            //C(0,i)=0.6f;C(1,i)=0.4f;C(2,i)=0.2f;
        C(0,i)=0.6f;C(1,i)=0.4f;C(2,i)=0.2f;
    }
	UV.resize(2,F.rows());
	UV<<0.75,0.5 ,0.5,0.75,0.5,0.75,0.5 ,0.75,0.75,0.5, 0.75,0.5,0.5,0.75,0.5 ,0.75,0.75,0.50,0.50,0.75,0.50,0.75,0.75,0.50,0.75,0.75,1.0, 0.75 ,1.0 ,1.0,0.50,0.50,0.25,0.25,0.50,0.25,
	0.25,0.25,0.5,0.25,0.5,0.5 ,0.75,0.75,0.5 ,0.75,0.5 ,0.5,1.0,1.0 ,0.75,1.0 ,0.75,0.75,0.25,0.25,0.0 ,0.25,0.0 ,0.0 ,0.25,0.50,0.25,0.50,0.50,0.25,0.25,0.0 ,0.0, 0.0 ,0.25,0.25;
    calculate_norm(light,box, F, barycenter,N_V);
    for(int i=0;i<3;i++){
        for(int j=0;j<box.rows();j++){
            center(i)+=box(j,i);
        }
        center(i) = center(i)/box.rows();
    }
    rotate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    to_center<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    to_center(0,3)=-center(0);
    to_center(1,3)=-center(1);
    to_center(2,3)=-center(2);
    to_center_mat.push_back(to_center);
    rotate_mat.push_back(rotate);
    translate_mat.push_back(translate);
    scale_mat.push_back(scale);
    obj_size.push_back(F.rows());

    add_vbo(objs,V);
	add_vbo(objs_uv,UV);
    add_vbo(objs_color,C);
    add_vbo(objs_normals,N_V);
}

#endif

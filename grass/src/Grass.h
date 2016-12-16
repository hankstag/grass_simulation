#ifndef GRASS_H
#define GRASS_H

#include <Eigen/Core>
#include <vector>
#include <time.h>
#include "model.h"
#include "Helpers.h"
using namespace std;
using namespace Eigen;

class Grass{
public:
    MatrixXf ctrl_pts; // control points coord 4*3
    MatrixXf sta_pos; // static position of control points
    MatrixXf velocity;
    MatrixXf acceleration;
	Vector3f clr;
    vector<Vector3f> segments; // Vector3f represent the segments between control points
    vector<double> ks; // sprint constants for last 3 control point
    Vector3f mass; // mass of each control point
    int index; // index in vector<vbo>
	float orient;
    Grass(Vector3f v,int index);
    void update(vector<VertexBufferObject>& objs, Vector3f& light, float winx,float winy);
    void get_sample_pts(Vector3f& light,int num_pt, MatrixXf& sample_pts, MatrixXf& normal, MatrixXf& color);
    void draw(
        vector<VertexBufferObject>& objs,
        vector<VertexBufferObject>& objs_color,
        vector<VertexBufferObject>&,
        Eigen::Vector3f& light,
        vector<int>& objs_size,
        vector<Matrix4f>& translate_mat,
        vector<Matrix4f>& to_center_mat,
        vector<Matrix4f>& scale_mat,
        vector<Matrix4f>& rotate_mat
    );
};

#endif

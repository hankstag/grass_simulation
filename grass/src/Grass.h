#ifndef GRASS_H
#define GRASS_H

#include <Eigen/Core>
#include <vector>
#include "model.h"
#include "Helpers.h"
using namespace std;
using namespace Eigen;

class Grass{
public:
    Eigen::MatrixXf ctrl_pts; // control points coord 4*3
    Eigen::MatrixXf velocity;
    Eigen::MatrixXf acceleration;
    Grass();
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

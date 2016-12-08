#ifndef GRASS_H
#define GRASS_H

#include <Eigen/Core>
#include <vector>
#include "Helpers.h"
using namespace std;

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
        vector<VertexBufferObject>&,
        vector<int>& opts
    );
};

#endif

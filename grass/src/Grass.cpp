#include "Grass.h"
#include <iostream>

Grass::Grass(){
    std::cout<<"init grass"<<std::endl;
    ctrl_pts.resize(3,5);
    ctrl_pts.col(0)<<0,0,-2;
    ctrl_pts.col(1)<<0,0,0;
    ctrl_pts.col(2)<<-1,-1,1;
    ctrl_pts.col(3)<<-2,-2,2;
    ctrl_pts.col(4)<<-3,-3,3;
    cout<<ctrl_pts<<endl;
}


void Grass::draw(
    vector<VertexBufferObject>& objs,
    vector<VertexBufferObject>& objs_color,
    vector<VertexBufferObject>& objs_normals,
    Vector3f& light,
    vector<int>& objs_size,
    vector<Matrix4f>& translate_mat,
    vector<Matrix4f>& to_center_mat,
    vector<Matrix4f>& scale_mat,
    vector<Matrix4f>& rotate_mat
){
    // pass the grass in as a new vbo
    float x,y,z;
	int j = 0;
    int count = 0;
	float width = 0.4f / 66;
	Eigen::MatrixXf color(3,136);
    Eigen::MatrixXf sample_pts(3,136);
    Eigen::MatrixXf normal(3,136);
    Matrix4f scale(4,4);
    scale<<0.1,0,0,0,0,0.1,0,0,0,0,-0.1,0,0,0,0,1;
    Matrix4f translate;
    translate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    Matrix4f rotate;
    rotate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    Matrix4f to_center;
    to_center<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
	for(int i = 1; i < 3; i++){
		for(double t = 0; t < 1; t += .03){
			x = catmullRomSpline(t,ctrl_pts(0,i-1),ctrl_pts(0,i),ctrl_pts(0,i+1), ctrl_pts(0,i+2));
			y = catmullRomSpline(t,ctrl_pts(1,i-1),ctrl_pts(1,i),ctrl_pts(1,i+1), ctrl_pts(1,i+2));
			z = catmullRomSpline(t,ctrl_pts(2,i-1),ctrl_pts(2,i),ctrl_pts(2,i+1), ctrl_pts(2,i+2));
			//printf("%f,%f,%f\n", g->p[1][0],g->p[1][1] , g->p[1][2]);
			// glColor3f(g->c[0][0] - col[0]*j,g->c[0][1] - col[1]*j,g->c[0][2] - col[2]*j);
            sample_pts.col(count)<<x,y,z;
            normal.col(count)<<(light-sample_pts.col(count)).normalized();
            color.col(count)<<0.0f,1.0f,0.0f;
            count++;
            sample_pts.col(count)<<x+(.5 - width*j),y+(.5 - width*j),z;
            normal.col(count)<<(light-sample_pts.col(count)).normalized();
            color.col(count)<<0.0f,1.0f,0.0f;
            count++;
			// glVertex3f(x,y,z);
			// glVertex3f(x+(.5 - width*j)*g->orient,y,z+(.5 - width*j)*(1.0f-g->orient));
			j++;
		}
	}
    to_center_mat.push_back(to_center);
    rotate_mat.push_back(rotate);
    translate_mat.push_back(translate);
    scale_mat.push_back(scale);

    VertexBufferObject vbo;
    vbo.init();
    vbo.update(sample_pts);
    objs.push_back(vbo);

    vbo.init();
    vbo.update(color);
    objs_color.push_back(vbo);

    vbo.init();
    vbo.update(normal);
    objs_normals.push_back(vbo);

}

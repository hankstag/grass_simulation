#include "Grass.h"
#include <iostream>

Grass::Grass(Vector3f root_pos, int index){
    std::cout<<"init grass"<<std::endl;
    ks.clear();
    this->index = index;
    ks.push_back(7);
    ks.push_back(5);
    ks.push_back(3);
    mass<<1.0,0.5,0.25;
	orient = float(rand())/RAND_MAX;
    ctrl_pts.resize(3,5);
	clr<<((float)rand()) / RAND_MAX,1.0 - (((float)rand()) / RAND_MAX)/4.0f, ((float)rand()) / RAND_MAX / 4.0f;
    ctrl_pts.col(0)<<root_pos(0),root_pos(1),root_pos(2)-2;
    ctrl_pts.col(1)<<root_pos;
    ctrl_pts.col(2)<<root_pos(0)-1 + (((float)rand())/RAND_MAX*2), root_pos(1) - 1 + (((float)rand())/RAND_MAX*2), root_pos(2)+ 1 + (((float)rand())/RAND_MAX*2);
    ctrl_pts.col(3)<<root_pos(0)-2 + (((float)rand())/RAND_MAX*2), root_pos(1) - 2 + (((float)rand())/RAND_MAX*2), root_pos(2)+ 2 + (((float)rand())/RAND_MAX*2);
    ctrl_pts.col(4)<<root_pos(0)-3 + (((float)rand())/RAND_MAX*2), root_pos(1) - 3 + (((float)rand())/RAND_MAX*2), root_pos(2)+ 3 + (((float)rand())/RAND_MAX);
    sta_pos=ctrl_pts;
    // the first two ctrl_pts are not movable
    segments.clear();
    segments.push_back(ctrl_pts.col(2)-ctrl_pts.col(1));
    segments.push_back(ctrl_pts.col(3)-ctrl_pts.col(2));
    segments.push_back(ctrl_pts.col(4)-ctrl_pts.col(3));

    // only move the top 3 control points
    acceleration.resize(3,3);
    velocity.resize(3,3);
    acceleration = MatrixXf::Zero(3,3);
    velocity = MatrixXf::Zero(3,3);
}

// update position of control points based on the wind
void Grass::update(
    vector<VertexBufferObject>& objs,
    Vector3f& light,
    float wind_x,
    float wind_y
){

    MatrixXf vn(3,3),pn(3,3);
    // displacement pt 2
    MatrixXf dspmt(3,3);
    // dspmt.col(0)<<(ctrl_pts.col(2)-sta_pos.col(1))-segments[0];
    // dspmt.col(1)<<(ctrl_pts.col(3)-sta_pos.col(2))-segments[1];
    // dspmt.col(2)<<(ctrl_pts.col(4)-sta_pos.col(3))-segments[2];
	dspmt.col(0)<<(ctrl_pts.col(2)-sta_pos.col(2));
    dspmt.col(1)<<(ctrl_pts.col(3)-sta_pos.col(3));
    dspmt.col(2)<<(ctrl_pts.col(4)-sta_pos.col(4));
    MatrixXf force(3,3);
    hooke_law(dspmt,ks,force);
    for(int i=0;i<3;i++){
        force(0,i)+=wind_x;
        force(1,i)+=wind_y;
        acceleration.col(i)<<force.col(i)/mass(i);
    }
	euler(ctrl_pts.block(0,2,3,3), 0.02, velocity, acceleration, vn, pn);
    velocity = vn;
    // how to make sure the length of each segments stays the same?
    Vector3f new_seg, pn_tmp;

	new_seg = pn.col(0) - ctrl_pts.col(1);
    pn.col(0) = ctrl_pts.col(1) + new_seg.normalized()*(segments[0].norm());
	velocity.col(0) = velocity.col(0)-new_seg.normalized()*((velocity.col(0)).dot(new_seg))/new_seg.norm();

	new_seg = pn.col(1) - pn.col(0);
    pn.col(1) = pn.col(0) + new_seg.normalized()*(segments[1].norm());
	velocity.col(1) = velocity.col(1)-new_seg.normalized()*((velocity.col(1)).dot(new_seg))/new_seg.norm();

	new_seg = pn.col(2) - pn.col(1);
    pn.col(2) = pn.col(1) + new_seg.normalized()*(segments[2].norm());
	velocity.col(2) = velocity.col(2)-new_seg.normalized()*((velocity.col(2)).dot(new_seg))/new_seg.norm();

	ctrl_pts.block(0,2,3,3)<<pn;
    int num_pt = 100;
    Eigen::MatrixXf color(3,num_pt*4);
    Eigen::MatrixXf sample_pts(3,num_pt*4);
    Eigen::MatrixXf normal(3,num_pt*4);
    get_sample_pts(light, num_pt, sample_pts, normal, color);
    objs[index].update(sample_pts);
}

void Grass::get_sample_pts(Vector3f& light, int num_pt, MatrixXf& sample_pts, MatrixXf& normal, MatrixXf& color){
    // pass the grass in as a new vbo
    float x,y,z;
	int j = 0;
    int count = 0;
	float width = 0.4f / (num_pt*2);
    int t = 0;
	for(int i = 1; i < 3; i++){
		for(int t = 0; t < num_pt; t++){
            double p = 1.0*t/num_pt;
			x = catmullRomSpline(p,ctrl_pts(0,i-1),ctrl_pts(0,i),ctrl_pts(0,i+1), ctrl_pts(0,i+2));
			y = catmullRomSpline(p,ctrl_pts(1,i-1),ctrl_pts(1,i),ctrl_pts(1,i+1), ctrl_pts(1,i+2));
			z = catmullRomSpline(p,ctrl_pts(2,i-1),ctrl_pts(2,i),ctrl_pts(2,i+1), ctrl_pts(2,i+2));
            sample_pts.col(count)<<x,y,z;
            normal.col(count)<<(light-sample_pts.col(count)).normalized();
            color.col(count)=clr;
            count++;
            sample_pts.col(count)<<x+(.5 - width*j)*orient,y+(.5 - width*j)*(1-orient),z;
            normal.col(count)<<(light-sample_pts.col(count)).normalized();
            color.col(count)=clr;
            count++;
			j++;
		}
	}
}

void Grass::draw(
	vector<VertexBufferObject>& objs,
    vector<VertexBufferObject>& objs_uv,
    vector<VertexBufferObject>& objs_color,
    vector<VertexBufferObject>& objs_normals,
    Vector3f& light,
    vector<int>& obj_size,
    vector<Matrix4f>& translate_mat,
    vector<Matrix4f>& to_center_mat,
    vector<Matrix4f>& scale_mat,
    vector<Matrix4f>& rotate_mat
){
    int num_pt = 100;
    Matrix4f scale(4,4);
    Eigen::MatrixXf color(3,num_pt*4);
    Eigen::MatrixXf sample_pts(3,num_pt*4);
    Eigen::MatrixXf normal(3,num_pt*4);
    scale<<0.1,0,0,0,0,0.1,0,0,0,0,-0.2,0,0,0,0,1;
    Matrix4f translate;
    translate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    Matrix4f rotate;
    rotate<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    Matrix4f to_center;
    to_center<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    get_sample_pts(light, num_pt, sample_pts, normal, color);
    to_center_mat.push_back(to_center);
    rotate_mat.push_back(rotate);
    translate_mat.push_back(translate);
    scale_mat.push_back(scale);
	Eigen::MatrixXf UV = Eigen::MatrixXf::Zero(2,color.cols());
	for(int i=0;i<2;i++)
		for(int j=0;j<UV.cols();j++)
			UV(i,j) -= 1;
    VertexBufferObject vbo;
    vbo.init();
    vbo.update(sample_pts);
    objs.push_back(vbo);

	vbo.init();
	vbo.update(UV);
	objs_uv.push_back(vbo);

    vbo.init();
    vbo.update(color);
    objs_color.push_back(vbo);

    vbo.init();
    vbo.update(normal);
    objs_normals.push_back(vbo);

    obj_size.push_back(num_pt*4);

}

#include <math.h>
#include "model.h"
#include <iostream>

//catmull-rom spline algorithm
float catmullRomSpline(float x, float v0,float v1,
				float v2,float v3) {

	float c1,c2,c3,c4;

	c1 =  	      M12*v1;
	c2 = M21*v0          + M23*v2;
	c3 = M31*v0 + M32*v1 + M33*v2 + M34*v3;
	c4 = M41*v0 + M42*v1 + M43*v2 + M44*v3;

	return(((c4*x + c3)*x +c2)*x + c1);

}

void hooke_law(MatrixXf dspmt, vector<double>& k, MatrixXf& force){
	// k is the sprint constant
	// each row represents a spring
	for(int i=0;i<dspmt.cols();i++){
		force.col(i) << -k[i]*dspmt.col(i);
	}
}

//Euler integration
void euler(MatrixXf pos, float h, MatrixXf v, MatrixXf a, MatrixXf& vn, MatrixXf& pn){
	// each row of pos represents a contorl point
	for(int i=0;i<pos.cols();i++){
		pn.col(i) = pos.col(i) + h*v.col(i) + (pow(h,2) * a.col(i))/2;
		vn.col(i) = v.col(i) + h*a.col(i);
		// std::cout<<i<<std::endl;
		// std::cout<<"v"<<std::endl;
		// std::cout<<v.col(i)<<std::endl;
		// std::cout<<"pn"<<std::endl;
		// std::cout<<pn.col(i)<<std::endl;
		// std::cout<<"a"<<std::endl;
		// std::cout<<a.col(i)<<std::endl;
	}
}

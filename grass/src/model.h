
#ifndef MODEL_H
#define MODEL_H
#include <Eigen/Core>
#include <vector>
using namespace Eigen;
using namespace std;
/* Coefficients for Matrix M */
#define M11	 0.0
#define M12	 1.0
#define M13	 0.0
#define M14	 0.0
#define M21	-0.5
#define M22	 0.0
#define M23	 0.5
#define M24	 0.0
#define M31	 1.0
#define M32	-2.5
#define M33	 2.0
#define M34	-0.5
#define M41	-0.5
#define M42	 1.5
#define M43	-1.5
#define M44	 0.5

float catmullRomSpline(float x, float v0,float v1,float v2,float v3);
void euler(MatrixXf pos, float h, MatrixXf v, MatrixXf a, MatrixXf& vn, MatrixXf& pn);
void hooke_law(MatrixXf dspmt, vector<double>& k, MatrixXf& force);

#endif

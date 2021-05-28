#ifndef EKF_ROT_H
#define EKF_ROT_H

#include <Eigen/Geometry>
#include "utils/QuaternionAlgebra.h"
#include <ctime>
#include <string>
#include <chrono>

using namespace Eigen;
using namespace std;

class KalmanRot
{
public:
	KalmanRot() {};
	KalmanRot(MatrixXd xin, MatrixXd Pin,  MatrixXd Qin, MatrixXd Rin,int id);

	int filterId;
	MatrixXd x;
	MatrixXd P;
	MatrixXd Q;
	MatrixXd R;
	MatrixXd K;
	MatrixXd A;
	MatrixXd C;	
	MatrixXd I;
	MatrixXd lastY;
	double lastSec;
	void update(MatrixXd y,double deltaT);
	void update(MatrixXd y);	
	MatrixXd getQuaternion();
	double averageDt;
private:
	double x0;
	double x1;
	double x2;
	double x3;
	double x4;
	double x5;
	double x6;
	double x7;
	double x8;
	double x9;		
	double normr;
	//std::clock_t lastTime;
	//std::clock_t currentTime;
	chrono::time_point<chrono::system_clock> chronoStart, chronoEnd;
	chrono::duration<double> elapsed_seconds;
	double dt,dt2,norm;
	MatrixXd quat_block,xquat;	
	long count;
		
	
};

#endif

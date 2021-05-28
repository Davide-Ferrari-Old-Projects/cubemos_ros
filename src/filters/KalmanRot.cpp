#include "filters/KalmanRot.h"
#include <iostream>


KalmanRot::KalmanRot(MatrixXd xin, MatrixXd Pin, MatrixXd Qin,MatrixXd Rin,int id)
{
	
	filterId = id;	
	
	x = xin;
	Q = Qin;
	P = Pin;
	R = Rin;

	lastY = MatrixXd::Zero(4,1);
	
	A = MatrixXd::Zero(10, 10);
	C = MatrixXd::Zero(4,10);
	C.block<4,4>(0,0) = MatrixXd::Identity(4,4);
	

	quat_block = MatrixXd::Zero(4,10);
	xquat = MatrixXd::Zero(4,1);


	I = MatrixXd::Identity(10, 10);

	averageDt = 0.0;
	count = 1l;
	
	//lastTime = clock();
	chronoStart = chrono::system_clock::now();
	lastSec = 0;
	
	
}

void KalmanRot::update(MatrixXd y)
{
	//currentTime = clock();
	//dt = (currentTime - lastTime)/(double)(CLOCKS_PER_SEC);
	//lastTime = currentTime;

	chronoEnd = chrono::system_clock::now();
	elapsed_seconds = chronoEnd - chronoStart;
	dt = elapsed_seconds.count();
	chronoStart = chrono::system_clock::now();		


	update(y,dt);	

}

void KalmanRot::update(MatrixXd y, double deltaT)
{
	
	//lastTime = clock();
	//lastSec = lastTime/(double)(CLOCKS_PER_SEC);
	
	dt = deltaT;
	lastSec = deltaT + lastSec;
	lastY = y;

	averageDt = (averageDt*(count-1) + dt)/count; 
	count++;


	A = MatrixXd::Zero(10, 10);
	A.block<3, 3>(4, 4) = MatrixXd::Identity(3, 3);
	A.block<3, 3>(7, 7) = MatrixXd::Identity(3, 3);
	A.block<3, 3>(4, 7) = MatrixXd::Identity(3, 3) * dt;

	dt2 = dt*dt;


	x0 = x(0, 0); x1 = x(1, 0); x2 = x(2, 0); x3 = x(3, 0); x4 = x(4, 0);
	x5 = x(5, 0); x6 = x(6, 0); x7 = x(7, 0); x8 = x(8, 0); x9 = x(9, 0);

	quat_block << 1 - (dt2*(x4*x4 / 2 + x5*x5 / 2 + x6*x6 / 2)) / 4, -(dt*x4) / 2 - (dt2*x7) / 4, -(dt*x5) / 2 - (dt2*x8) / 4, -(dt*x6) / 2 - (dt2*x9) / 4, -(dt*x1) / 2 - (dt2*x0*x4) / 4, -(dt*x2) / 2 - (dt2*x0*x5) / 4, -(dt*x3) / 2 - (dt2*x0*x6) / 4, -(dt2*x1) / 4, -(dt2*x2) / 4, -(dt2*x3) / 4,
	(dt*x4) / 2 + (dt2*x7) / 4, 1 - (dt2*(x4*x4 / 2 + x5*x5 / 2 + x6*x6 / 2)) / 4, -(dt*x6) / 2 - (dt2*x9) / 4, (dt*x5) / 2 + (dt2*x8) / 4, (dt*x0) / 2 - (dt2*x1*x4) / 4, (dt*x3) / 2 - (dt2*x1*x5) / 4, -(dt*x2) / 2 - (dt2*x1*x6) / 4, (dt2*x0) / 4, (dt2*x3) / 4, -(dt2*x2) / 4,
	(dt*x5) / 2 + (dt2*x8) / 4, (dt*x6) / 2 + (dt2*x9) / 4, 1 - (dt2*(x4*x4 / 2 + x5*x5 / 2 + x6*x6 / 2)) / 4, -(dt*x4) / 2 - (dt2*x7) / 4, -(dt*x3) / 2 - (dt2*x2*x4) / 4, (dt*x0) / 2 - (dt2*x2*x5) / 4, (dt*x1) / 2 - (dt2*x2*x6) / 4, -(dt2*x3) / 4, (dt2*x0) / 4, (dt2*x1) / 4,
	(dt*x6) / 2 + (dt2*x9) / 4, -(dt*x5) / 2 - (dt2*x8) / 4, (dt*x4) / 2 + (dt2*x7) / 4, 1 - (dt2*(x4*x4 / 2 + x5*x5 / 2 + x6*x6 / 2)) / 4, (dt*x2) / 2 - (dt2*x3*x4) / 4, -(dt*x1) / 2 - (dt2*x3*x5) / 4, (dt*x0) / 2 - (dt2*x3*x6) / 4, (dt2*x2) / 4, -(dt2*x1) / 4, (dt2*x0) / 4;



	A.block<4, 10>(0, 0) = quat_block;

	x(0, 0) = x0 - (dt*(x1*x4 + x2*x5 + x3*x6)) / 2 - (dt2*(x4*((x0*x4) / 2 - (x2*x6) / 2 + (x3*x5) / 2) + x5*((x0*x5) / 2 + (x1*x6) / 2 - (x3*x4) / 2) + x6*((x0*x6) / 2 - (x1*x5) / 2 + (x2*x4) / 2) + x1*x7 + x2*x8 + x3*x9)) / 4;
	x(1, 0) = x1 + (dt*(x0*x4 - x2*x6 + x3*x5)) / 2 + (dt2*(x5*((x0*x6) / 2 - (x1*x5) / 2 + (x2*x4) / 2) - x4*((x1*x4) / 2 + (x2*x5) / 2 + (x3*x6) / 2) - x6*((x0*x5) / 2 + (x1*x6) / 2 - (x3*x4) / 2) + x0*x7 - x2*x9 + x3*x8)) / 4;
	x(2, 0) = x2 + (dt*(x0*x5 + x1*x6 - x3*x4)) / 2 - (dt2*(x4*((x0*x6) / 2 - (x1*x5) / 2 + (x2*x4) / 2) + x5*((x1*x4) / 2 + (x2*x5) / 2 + (x3*x6) / 2) - x6*((x0*x4) / 2 - (x2*x6) / 2 + (x3*x5) / 2) - x0*x8 - x1*x9 + x3*x7)) / 4;
	x(3, 0) = x3 + (dt*(x0*x6 - x1*x5 + x2*x4)) / 2 + (dt2*(x4*((x0*x5) / 2 + (x1*x6) / 2 - (x3*x4) / 2) - x5*((x0*x4) / 2 - (x2*x6) / 2 + (x3*x5) / 2) - x6*((x1*x4) / 2 + (x2*x5) / 2 + (x3*x6) / 2) + x0*x9 - x1*x8 + x2*x7)) / 4;
	x(4, 0) = x4 + x7*dt;
	x(5, 0) = x5 + x8*dt;
	x(6, 0) = x6 + x9*dt;
	x(7, 0) = x7;
	x(8, 0) = x8;
	x(9, 0) = x9;



	xquat = x.block<4, 1>(0, 0);
	norm = xquat.norm();
	x.block<4, 1>(0, 0) = x.block<4, 1>(0, 0) / norm;
	

	


	P = A*P*(A.transpose()) + Q;
	
	K = P*(C.transpose()) * ((C*P*(C.transpose()) + R).inverse());
	
	x = x + K*(y - C*x);	
	
	P = (I - K*C)*P;

	//std::cout<<"ang dt: "<<dt<<" y: "<< y(0,0)<<" x: "<<x(0,0)<< " meanDt "<< averageDt <<std::endl;

	//currentTime = clock();

	//if (filterId == 13)
	//std::cout<<"quat: "<<y(0,0)<<"  "<< y(1,0)<<" "<<y(2,0)<< "  "<< y(3,0) <<std::endl;
		


}



MatrixXd KalmanRot::getQuaternion()
{
	return x.block<4, 1>(0, 0);
}


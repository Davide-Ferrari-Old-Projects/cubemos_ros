#ifndef KALMAN_LIN_H
#define KALMAN_LIN_H

#include <Eigen/Geometry>
#include <ctime>
#include <string>
#include <fstream>
#include <chrono>

using namespace Eigen;
using namespace std;

class KalmanLin
{

  private: 
	//clock_t lastTime;
	//clock_t currentTime;
	chrono::time_point<chrono::system_clock> chronoStart, chronoEnd;
	chrono::duration<double> elapsed_seconds;
	double dt;
	long count;
  public:
     	 KalmanLin(){};
	 KalmanLin(MatrixXd xin,MatrixXd Pin,MatrixXd Qin,MatrixXd Rin,int id);	 
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
	 int filterId;		 
	 double averageDt;
	 void update(MatrixXd y);
	 void update(MatrixXd y,double deltaT);	

};

#endif

#ifndef MEDIUM_FILT_H
#define MEDIUM_FILT_H

#include <Eigen/Geometry>
#include <ctime>
#include <string>
#include <fstream>
#include <chrono>

using namespace Eigen;
using namespace std;

class MediumFilt
{

	private: 
		//clock_t lastTime;
		//clock_t currentTime;
		chrono::time_point<chrono::system_clock> chronoStart, chronoEnd;
		chrono::duration<double> elapsed_seconds;
		double dt;
		long count;
	public:
		MediumFilt(){};
		MediumFilt(MatrixXd xin,MatrixXd Pin,MatrixXd Qin,MatrixXd Rin,int id);
		MediumFilt(int n);	 
		MatrixXd x;	 
		MatrixXd last_pos;
		MatrixXd data;
		int data_collect;
		int N;
		bool enough_data;
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

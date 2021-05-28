#include "filters/MediumFilt.h"
#include <iostream>

MediumFilt::MediumFilt(MatrixXd xin,MatrixXd Pin,MatrixXd Qin,MatrixXd Rin,int id)
{

		
		x=xin;
		Q=Qin;		
		P=Pin;
		R=Rin;
		
		A = MatrixXd::Identity(9,9);	
		lastY = MatrixXd::Zero(3,1);			
		
		C = MatrixXd::Identity(3, 9);
		C.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

		I = MatrixXd::Identity(9,9);
		averageDt = 0.0;
		count = 1l;
		filterId = id;

		//lastTime = clock();
		chronoStart = chrono::system_clock::now();
		lastSec = 0;
		enough_data = false;

		
}

MediumFilt::MediumFilt(int n)
{
	x = MatrixXd::Zero(9,1);
	last_pos = MatrixXd::Zero(3,n);
	data = MatrixXd::Zero(3,n);
	data_collect = 0;
	N = n;

}

void MediumFilt::update(MatrixXd y)
{
		
	data.block<3,1>(0,data_collect) = y;
	data_collect++;

	if(data_collect == N)
	{
		enough_data = true;
		data_collect = 0;
	}

	if(enough_data)
	{
		chronoEnd = chrono::system_clock::now();
		elapsed_seconds = chronoEnd - chronoStart;
		dt = elapsed_seconds.count();	
		last_pos = x.block<3,1>(0,0);
		x.block<3,1>(0,0) = data.rowwise().sum()/N;
		x.block<3,1>(3,0) = (x.block<3,1>(0,0) - last_pos)/dt;
		chronoStart = chrono::system_clock::now();
	}
}

void MediumFilt::update(MatrixXd y,double deltaT)
{

		//lastTime = clock();		
		//lastSec = lastTime/(double)(CLOCKS_PER_SEC);

		lastY = y;
		lastSec = deltaT + lastSec;


		dt = deltaT;
		averageDt = (averageDt*(count-1) + dt)/count; 
		count++;
		
		
		A = MatrixXd::Zero(9, 9);
		A.topRightCorner(6, 6) = MatrixXd::Identity(6, 6) * dt;
		A.topRightCorner(3, 3) = MatrixXd::Identity(3, 3) * dt * dt * 0.5;
		A = A + MatrixXd::Identity(9, 9);

		/** PREDIZIONE **/
		//stato predetto
		x = A * x;
		//covarianza predetta
		P = A * P * (A.transpose()) + Q;

		/** AGGIORNAMENTO **/
		//guadagno di kalman
		K = P * (C.transpose()) * ((C * P * (C.transpose()) + R).inverse());
		//aggiornamento dello stato
		x = x + K * (y - C*x); //y-Cx: innovazione
		//aggiornamento covarianza
		P = (I - K * C) * P;

		// if (filterId == 13)
		// std::cout<<"lin dt: "<<dt<<" y2: "<< y(1,0)<<" x2: "<<x(1,0)<< " meanDt "<< averageDt <<std::endl;
		

		//currentTime = clock();

		


}



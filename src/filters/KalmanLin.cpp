#include "filters/KalmanLin.h"
#include <iostream>

KalmanLin::KalmanLin(MatrixXd xin,MatrixXd Pin,MatrixXd Qin,MatrixXd Rin,int id)
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

		
}

void KalmanLin::update(MatrixXd y)
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

void KalmanLin::update(MatrixXd y,double deltaT)
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



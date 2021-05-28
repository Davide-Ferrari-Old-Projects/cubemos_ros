#ifndef CUBEMOS_ROS_H
#define CUBEMOS_ROS_H

#include <iomanip>
#include <map>
#include <string>
#include <iterator>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <algorithm>
#include <Eigen/Geometry>
#include "utils/QuaternionAlgebra.h"
#include "filters/KalmanRot.h"
#include "filters/KalmanLin.h"
#include "filters/MediumFilt.h"
#include <mutex>
#include <chrono>


using namespace Eigen;
using namespace std;

struct FilteredJoint {
	
	int userID;
	int index;

	MatrixXd position;
	MatrixXd rotationMatrix;
	MatrixXd quaternion;	
	MatrixXd linearVel;
	MatrixXd linearAcc;
	MatrixXd angularVel;
	MatrixXd angularAcc;	

	MatrixXd linMeas;
	MatrixXd angMeas;
	double linT;
	double angT;
	
	float confidence;
	bool tracked;

};

class CubemosROS {

	using CUBEMOS_SKEL_Buffer_Ptr = std::unique_ptr<CM_SKEL_Buffer, void (*)(CM_SKEL_Buffer*)>;

	struct cmPoint {
		float color_pixel[2];
		float point3d[3];
		std::string to_string() const
		{
			char buffer[100];
			int cx = snprintf(buffer, 100, "(%.2f, %.2f, %.2f)", point3d[0], point3d[1], point3d[2]);
			return std::string(buffer);
		}
	};

	struct Joints {
		float x;
		float y;
		float z;
	};

	private:
		
		// Nuitrack 
		// SkeletonTracker::Ptr skTrack;		
		// Skeleton currentSkeleton;
		// float confidenceThreshold;
		bool updating;

		// Filters
		std::string configPath;				
		vector<FilteredJoint> joints;
		vector<MediumFilt> MediumFilters;
		vector<KalmanLin> kalmanLinFilters;
		vector<KalmanRot> kalmanAngFilters;
		MediumFilt mfilt;
		KalmanLin klin;
		KalmanRot kang;
		MatrixXd yLin,yAng,xLin,xAng,Qlin,Qang,Plin,Pang,Rlin,Rang;
		Matrix3d rot;
		Quaterniond quat,quatOld;
		double normQ;
		vector<int> trackedJoints;
		mutex mut;

		rs2::pipeline pipe;
		rs2::config cfg;
		rs2::context ctx;
		rs2::align *align_to_color;
		cv::Mat capturedFrame;
		const int nHeight = 192;
		CM_ReturnCode retCode;
		CM_SKEL_Handle* handle;
		CUBEMOS_SKEL_Buffer_Ptr *skeletonsPresent;
		CUBEMOS_SKEL_Buffer_Ptr *skeletonsLast;
		int frameCount;
		std::string fpsTest;
		std::chrono::time_point<std::chrono::system_clock> startTime;
		std::vector<Joints> skeleton;


		// //logging
		// vector<string> fileNames;		
		// vector<shared_ptr<ofstream>> logFiles;
		chrono::time_point<chrono::system_clock> chronoStart,chronoEnd;
		// chrono::duration<double> elapsed_seconds;
	

		// // Methods
		void initVariables();		
		int initCubemos();
		void initMediumFilter(int index);
		void initFilter(int index);
		// void logJoints(int index);
		// void onSkeletonUpdate(SkeletonData::Ptr skeletonData); // Callback for the skeleton data update event
		void update();
		void updateMediumFilter(int index);
		void updateFilter(int index);  // update filters		
		void initFilteredJoints(); // init nuitrackJoint vector
		void updateFilteredJoint(int index); // update nuitrackJoint vector
		cmPoint get_skeleton_point_3d(rs2::depth_frame const& depthFrame, int x, int y);
		CUBEMOS_SKEL_Buffer_Ptr create_skel_buffer();
		inline void renderSkeletons(const CM_SKEL_Buffer* skeletons_buffer, rs2::depth_frame const& depth_frame, cv::Mat& image);

	public:

		//Constructor
		CubemosROS();	

		// get joints
		vector<FilteredJoint> getFilteredJoints();

		
		
	

};







#endif

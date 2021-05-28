#include "nuitrack_skeleton_tracker_filtered/body.h"
#include "ros/ros.h"
#include <sstream>
#include "nuitrackWrap/nuitrack_tracker_filtered.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std; 

// map a nuitrack joint to body msg
void mapJoint(nuitrack_skeleton_tracker_filtered::body* msg,nuitrackFilteredJoint joint) 
{

	geometry_msgs::TransformStamped tmp;
	geometry_msgs::TwistStamped tmpTwist;
	geometry_msgs::AccelStamped tmpAcc;


      
	// Header
	tmp.header.stamp = ros::Time::now();
	tmpTwist.header.stamp = ros::Time::now();
	tmpAcc.header.stamp = ros::Time::now();
	

	// Position/Orientation
	tmp.transform.translation.x = joint.position(0,0);
	tmp.transform.translation.y = joint.position(1,0);
	tmp.transform.translation.z = joint.position(2,0);
	tmp.transform.rotation.x = joint.quaternion(0,0);
	tmp.transform.rotation.y = joint.quaternion(1,0);
	tmp.transform.rotation.z = joint.quaternion(2,0);
	tmp.transform.rotation.w = joint.quaternion(3,0);

	// Velocity
	tmpTwist.twist.linear.x = joint.linearVel(0,0);
	tmpTwist.twist.linear.y = joint.linearVel(1,0);
	tmpTwist.twist.linear.z = joint.linearVel(2,0);
	tmpTwist.twist.angular.x = joint.angularVel(0,0);
	tmpTwist.twist.angular.y = joint.angularVel(1,0);
	tmpTwist.twist.angular.z = joint.angularVel(2,0);

	// Accel
	tmpAcc.accel.linear.x = joint.linearAcc(0,0);
	tmpAcc.accel.linear.y = joint.linearAcc(1,0);
	tmpAcc.accel.linear.z = joint.linearAcc(2,0);
	tmpAcc.accel.angular.x = joint.angularAcc(0,0);
	tmpAcc.accel.angular.y = joint.angularAcc(1,0);
	tmpAcc.accel.angular.z = joint.angularAcc(2,0);

	

	// Debug
	/*if (joint.index == 2)
	{
		cout << "neck position x " << joint.position(0,0)<<endl;
		cout << "neck vel x " << joint.linearVel(0,0)<<endl;
		cout << "neck acc x " << joint.linearAcc(0,0)<<endl; 
	}*/



		
	// joint map
	switch (joint.index)
	{
		case (0): // none
		break;
		case (1): // head
			msg->head = tmp;
			msg->head_vel = tmpTwist;
			msg->head_acc = tmpAcc;
		break;
		case (2): // neck
			msg->neck = tmp;
			msg->neck_vel = tmpTwist;
			msg->neck_acc = tmpAcc;
		break;
		case (3): // torso			
			msg->torso = tmp;			
			msg->torso_vel = tmpTwist;
			msg->torso_acc = tmpAcc;
		break;
		case (4): // waist
		break;
		case (5): // left collar
		break;
		case (6): // left shoulder
			msg->left_shoulder = tmp;
			msg->left_shoulder_vel = tmpTwist;
			msg->left_shoulder_acc = tmpAcc;
		break;
		case (7): // left elbow
			msg->left_elbow = tmp;
			msg->left_elbow_vel = tmpTwist;
			msg->left_elbow_acc = tmpAcc;
		break;
		case (8): // left wrist	
		break;
		case (9):  // left hand
			msg->left_hand = tmp;
			msg->left_hand_vel = tmpTwist;
			msg->left_hand_acc = tmpAcc;
		break;
		case (10):  // left fingertip (not used)
		break;
		case (11): // right collar
		break;
		case (12):  // right shoulder
			msg->right_shoulder = tmp;
			msg->right_shoulder_vel = tmpTwist;
			msg->right_shoulder_acc = tmpAcc;
		break;
		case (13):  // right elbow
			msg->right_elbow = tmp;
			msg->right_elbow_vel = tmpTwist;
			msg->right_elbow_acc = tmpAcc;
		break;
		case (14):  // right wrist
		break;
		case (15):  // right hand
			msg->right_hand = tmp;
			msg->right_hand_vel = tmpTwist;
			msg->right_hand_acc = tmpAcc;
		break;
		case (16):  // right fingertip (not used)
		break;
		case (17): // left hip
			msg->left_hip = tmp;
			msg->left_hip_vel = tmpTwist;
			msg->left_hip_acc = tmpAcc;
		break;
		case (18): // left knee
			msg->left_knee = tmp;
			msg->left_foot_vel = tmpTwist;
			msg->left_foot_acc = tmpAcc;
		break;
		case (19): // left ankle
			msg->left_foot = tmp;
			msg->left_foot_vel = tmpTwist;
			msg->left_foot_acc = tmpAcc;
		break;
		case (20): // left foot (not used)
		break;
		case (21): // right hip
			msg->right_hip = tmp;
			msg->right_hip_vel = tmpTwist;
			msg->right_hip_acc = tmpAcc;
		break;
		case (22): // right knee
			msg->right_knee = tmp;
			msg->right_knee_vel = tmpTwist;
			msg->right_knee_acc = tmpAcc;
		break;
		case (23): // right ankle
			msg->right_foot = tmp;
			msg->right_foot_vel = tmpTwist;
			msg->right_foot_acc = tmpAcc;
		break;
		case (24): // right foot (not used)
		break;		

	}

}


// Main
int main(int argc, char **argv) 
{

	
	// ROS init
	ROS_INFO("node init.. ");
	ros::init(argc, argv, "nuitrack_tracker");	
	ros::NodeHandle n;
	ros::Publisher skeleton_pub = n.advertise<nuitrack_skeleton_tracker_filtered::body>("skeleton_topic", 1); 	
	ros::Rate loop_rate(60);

	
	// Nuitrack init
	ROS_INFO("nuitrack init.. ");
	NuitrackTrackerFilt nTrack;	
	vector<nuitrackFilteredJoint> nJoints;
	

  
	// Node loop
	ROS_INFO("Tracking loop starting.. ");
	while (ros::ok())
	{

		nuitrack_skeleton_tracker_filtered::body msg;
		nJoints=nTrack.getFilteredJoints();

		
		if (!nJoints.empty())
		{
			
			

			for (int i=0;i<nJoints.size();i++) 
			{	
				// Debug			
				//pos = nJoints[i].position;
				//rot = nJoints[i].rotationMatrix;
				//cout << "joint " << to_string(i) << " x: " << pos[0] << "r: " << rot[1][2] << "conf: " << nJoints[i].confidence << endl;

				// Map joint to body message
				if (nJoints[i].tracked)
				{
					//cout << "tracking joint " << to_string(i) << endl;
					mapJoint(&msg,nJoints[i]);
				}
				else
				{
					//cout << "joint " << to_string(i) << " not found " << endl;
				}				
				
			}
			
		}
		

	    		


		
		// Publish message
		skeleton_pub.publish(msg);

		// Spin and sleep
		ros::spinOnce();
		loop_rate.sleep();

	}

  	ROS_INFO("Stop Tracking");

  	return 0;

}


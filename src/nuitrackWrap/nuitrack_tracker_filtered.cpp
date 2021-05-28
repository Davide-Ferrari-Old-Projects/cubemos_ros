#include "nuitrackWrap/nuitrack_tracker_filtered.hpp"


// Constructor
NuitrackTrackerFilt::NuitrackTrackerFilt() 
{

	initVariables();

	initFilteredJoints();

	cout << "initializing filters.." << endl;
	
	for (int i=0;i<joints.size();i++)
	{
		initFilter(i);
	}

	cout << "initializing nuitrack..." << endl;
	try
	{
		initNuitrack();
	}
	catch (const Exception& e)
	{
		cerr << "Can not initialize the tracker (ExceptionType: " << e.type() << ")" << endl;
		return;
	}
	cout << "init ok" << endl;

	

	


}

void NuitrackTrackerFilt::initVariables()
{
	updating = false;
	configPath = "";
	//configPath = "/home/liralab/nuitrackCustomConfig/nuitrack.config";
	skTrack = NULL;
	confidenceThreshold = 0.7f;
	nuitrackFilteredJoint nJ;
	nJ.position = MatrixXd::Zero(3,1);
	nJ.linearVel = MatrixXd::Zero(3,1);
	nJ.linearAcc = MatrixXd::Zero(3,1);
	nJ.quaternion = MatrixXd::Zero(4,1);
	nJ.angularVel = MatrixXd::Zero(3,1);
	nJ.angularAcc = MatrixXd::Zero(3,1);
	nJ.linMeas = MatrixXd::Zero(3,1);
	nJ.angMeas = MatrixXd::Zero(4,1);
	
	for (int i=0;i<25;i++)
	{
		joints.push_back(nJ);
	}

	xLin = MatrixXd::Zero(9,1);
	Qlin = (VectorXd(9) << 1e-5,1e-5,1e-5,1e-2,1e-2,1e-2,1e-1,1e-1,1e-1).finished().asDiagonal();
	Rlin = (VectorXd(3) << 1e-6,1e-6,1e-7).finished().asDiagonal();
	Plin = (VectorXd(9) << 1,1,1,1,1,1,1,1,1).finished().asDiagonal();
	yLin = MatrixXd::Zero(3,1);

	xAng = MatrixXd::Zero(10,1);
	xAng(0,0) = 1.0;
	Qang = (VectorXd(10) << 1e-5,1e-5,1e-5,1e-5,1e-4,1e-4,1e-4,1e-2,1e-2, 1e-2).finished().asDiagonal();
	Rang = (VectorXd(4) << 4e-5,4e-5,4e-5,4e-5).finished().asDiagonal();
	Pang = (VectorXd(10) << 1,1,1,1,1,1,1,1,1,1).finished().asDiagonal();
	yAng = MatrixXd::Zero(4,1);
	rot  << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	quat = Quaterniond(0.0,0.0,0.0,0.0);
	quatOld = Quaterniond(0.0,0.0,0.0,0.0);	

	// trackedJoints.push_back(1); //head
	// trackedJoints.push_back(2); //neck
	// trackedJoints.push_back(3); //torso	
	// trackedJoints.push_back(4); //waist	
	// trackedJoints.push_back(5); //left collar	
	// trackedJoints.push_back(6); //left shoulder
	// trackedJoints.push_back(7); //left elbow
	// trackedJoints.push_back(8); //left wrist	
	// trackedJoints.push_back(9); //left hand	
	// trackedJoints.push_back(10); //left fingertip
	// trackedJoints.push_back(11); //right collar
	// trackedJoints.push_back(12); //right shoulder
	// trackedJoints.push_back(13); //right elbow
	// trackedJoints.push_back(14); //right wrist
	trackedJoints.push_back(15); //right hand
	// trackedJoints.push_back(16); //right fingertip
	// trackedJoints.push_back(17); //left hip
	// trackedJoints.push_back(18); //left knee
	// trackedJoints.push_back(19); //left ankle
	// trackedJoints.push_back(20); //left foot
	// trackedJoints.push_back(21); //right hip
	// trackedJoints.push_back(22); //right knee
	// trackedJoints.push_back(23); //right ankle
	// trackedJoints.push_back(24); //right foot
	
	// init log files

	for (int i=0;i<25;i++)
	{
		fileNames.push_back("/home/liralab/logs/joint" + to_string(i) + ".txt");		
		logFiles.push_back(make_shared<ofstream>(fileNames[i]));
		(*logFiles[i]) << "Time" << ";" << "Position X" << ";" << "Position Y" << ";" << "Position Z" << ";"
			<< "linVel X" << ";" << "linVel Y" << ";" << "linVel Z" << ";"
			<< "linAcc X" << ";" << "linAcc Y" << ";" << "linAcc Z" << ";"
			<< "Quaternion w" << ";" << "Quaternion x" << ";" << "Quaternion y" << ";" << "Quaternion z" << ";"
			<< "angVel X" << ";" << "angVel Y" << ";" << "angVel Z" << ";"
			<< "angAcc X" << ";" << "angAcc Y" << ";" << "angAcc Z" << ";"
			<< "linMeas X" << ";" << "linMeas Y" << ";" << "linMeas Z" << ";"
			<< "angMeas w" << ";" << "angMeas x" << ";" << "angMeas y" << ";" "angMeas z" << ";" "timeLin" << ";" "timeAng" << endl;	

	}

	chronoStart = chrono::system_clock::now();

	
}

void NuitrackTrackerFilt::initFilter(int index)
{

	string logFile = "/home/liralab/logs/jointKal" + to_string(index) + ".txt";
	string logFileAng = "/home/liralab/logs/jointKalAng" + to_string(index) + ".txt";

	klin = KalmanLin(xLin,Plin,Qlin,Rlin,index);
	kang = KalmanRot(xAng,Pang,Qang,Rang,index);

	
			
	if (kalmanLinFilters.size()<index+1) 
	{
		kalmanLinFilters.push_back(klin);
		kalmanAngFilters.push_back(kang);
	}
	else
	{
		kalmanLinFilters[index] = klin;
		kalmanAngFilters[index] = kang;
	}

	


}

void NuitrackTrackerFilt::updateFilter(int index)
{

	// skip update if index is not in trackedJoints vector
	
	if (find(trackedJoints.begin(),trackedJoints.end(),index) == trackedJoints.end()) return;

	

	// position measurements	
	
	yLin(0,0) = currentSkeleton.joints[index].real.x/1000.0; 
	//yLin(1,0) = currentSkeleton.joints[index].real.y/1000.0; 
	yLin(1,0) = -currentSkeleton.joints[index].real.y/1000.0;  // left-handed correction
	yLin(2,0) = currentSkeleton.joints[index].real.z/1000.0; 

	// orientation measurements	

	rot(0,0) = currentSkeleton.joints[index].orient.matrix[0];
	rot(0,1) = currentSkeleton.joints[index].orient.matrix[1];
	rot(0,2) = currentSkeleton.joints[index].orient.matrix[2];
				
	rot(1,0) = currentSkeleton.joints[index].orient.matrix[3];
	rot(1,1) = currentSkeleton.joints[index].orient.matrix[4];
	rot(1,2) = currentSkeleton.joints[index].orient.matrix[5];

	rot(2,0) = currentSkeleton.joints[index].orient.matrix[6];
	rot(2,1) = currentSkeleton.joints[index].orient.matrix[7];
	rot(2,2) = currentSkeleton.joints[index].orient.matrix[8];

	
	
	quat = Quaterniond(rot);
	quat = quat.normalized();
	
	normQ = sqrt(pow(quat.x()-quatOld.x(), 2.0) + pow(quat.y()-quatOld.y(), 2.0) + pow(quat.z()-quatOld.z(), 2.0) + pow(quat.w()-quatOld.w(), 2.0));
	if (normQ > 1.99) 
	{
		quat.x() = -quat.x();
		quat.y() = -quat.y();
		quat.z() = -quat.z();
		quat.w() = -quat.w();
	}
	
	
	
	quatOld = quat;

	//yAng(0,0) = quat.w();
	yAng(0,0) = -quat.w(); // left-handed correction
	yAng(1,0) = quat.x();
	//yAng(2,0) = quat.y();
	yAng(2,0) = -quat.y(); // left-handed correction
	yAng(3,0) = quat.z();
	

	kalmanLinFilters[index].update(yLin);	
	kalmanAngFilters[index].update(yAng);	

}


// Initialize (start updating thread)
void NuitrackTrackerFilt::initNuitrack()
{
    
    
    // Initialize Nuitrack
    Nuitrack::init(configPath);  

    /*Nuitrack::setConfigValue("Realsense2Module.Depth.Preset","5"); 
    Nuitrack::setConfigValue("Realsense2Module.Depth.RawWidth","640"); 
    Nuitrack::setConfigValue("Realsense2Module.Depth.RawHeight","480"); 
    Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessWidth","640"); 
    Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessHeight","480"); 
    Nuitrack::setConfigValue("Realsense2Module.Depth.FPS","30");  */
    
    // Create Skeleton tracker module, other required modules will be
    // created automatically
    skTrack = SkeletonTracker::create();

    // Connect onSkeleton callback to receive skeleton tracking data
    skTrack->connectOnUpdate(bind(&NuitrackTrackerFilt::onSkeletonUpdate,this,placeholders::_1));
    // Start Nuitrack
    Nuitrack::run();
    // Start updating thread
    updating = true;

    cout << "starting nuitrack thread..." << endl;
    thread updatingThread(bind(&NuitrackTrackerFilt::update,this));
    updatingThread.detach();
    

}


// Get nuitrack joint vector
vector<nuitrackFilteredJoint> NuitrackTrackerFilt::getFilteredJoints()
{	
	return joints;
}


// update callback
void NuitrackTrackerFilt::update()
{
	while(updating)
	{
		try
		{
			Nuitrack::waitUpdate(skTrack);
		}
		catch (LicenseNotAcquiredException& e)
       		{
		    cerr << "LicenseNotAcquired exception (ExceptionType: " << e.type() << ")" << endl;
		    //errorCode = EXIT_FAILURE;
		    break;
		}
		catch (const Exception& e)
		{
		    cerr << "Nuitrack update failed (ExceptionType: " << e.type() << ")" << endl;
		    //errorCode = EXIT_FAILURE;
		}
	}
	Nuitrack::release();

}
	

	
// Callback for the skeleton data update event
void NuitrackTrackerFilt::onSkeletonUpdate(SkeletonData::Ptr skeletonData)
{
	if (!skeletonData)
	{	
		//cout << "No skeleton data" << endl;		
		for (int i = 0;i<joints.size();i++)
		{
			joints[i].tracked = false;
			initFilter(i);
		}
		
	}
	else
	{
		auto userSkelets = skeletonData->getSkeletons();
		if (userSkelets.empty())
		{
			// No user skeleton, reinit filters			
			for (int i = 0;i<joints.size();i++)
			{
				//cout << "filter init 2" <<endl;
				joints[i].tracked = false;
				initFilter(i);
			}
		}
		else
		{
			// update filters
			currentSkeleton = userSkelets[0];
			for (int i=0;i<currentSkeleton.joints.size();i++)
			{

				if (currentSkeleton.joints[i].confidence > confidenceThreshold)
				{
					updateFilter(i);
					joints[i].tracked = true;					
				}	
				else
				{
					joints[i].tracked = false;
					initFilter(i);								
				}
			}

		}
	}

	
	for (int i=0;i<joints.size();i++)
	{
		updateFilteredJoint(i);	
		logJoints(i);	
		
	}
	
	    
}

void NuitrackTrackerFilt::logJoints(int index)
{

	chronoEnd = chrono::system_clock::now();
	elapsed_seconds = chronoEnd - chronoStart;	

	//cout << "log joint " << index << "file: " << fileNames[index] << endl;
	(*logFiles[index]) << elapsed_seconds.count() << ";" << joints[index].position(0,0) << ";" << joints[index].position(1,0) << ";" << joints[index].position(2,0) << ";"
			<< joints[index].linearVel(0,0) << ";" << joints[index].linearVel(1,0) << ";" << joints[index].linearVel(2,0) << ";"
			<< joints[index].linearAcc(0,0) << ";" << joints[index].linearAcc(1,0) << ";" << joints[index].linearAcc(2,0) << ";"
			<< joints[index].quaternion(0,0) << ";" << joints[index].quaternion(1,0) << ";" << joints[index].quaternion(2,0) << ";" << joints[index].quaternion(3,0) << ";"
			<< joints[index].angularVel(0,0) << ";" << joints[index].angularVel(1,0) << ";" << joints[index].angularVel(2,0) << ";"
			<< joints[index].angularAcc(0,0) << ";" << joints[index].angularAcc(1,0) << ";" << joints[index].angularAcc(2,0) << ";"
			<< joints[index].linMeas(0,0) << ";" << joints[index].linMeas(1,0) << ";" << joints[index].linMeas(2,0) << ";"
			<< joints[index].angMeas(0,0) << ";" << joints[index].angMeas(1,0) << ";" << joints[index].angMeas(2,0) << ";" << joints[index].angMeas(3,0) << ";" << joints[index].linT
			<< ";" << joints[index].angT << endl;
	//logFiles[index]->close();

}

void NuitrackTrackerFilt::initFilteredJoints()
{
	for (int i=0;i<joints.size();i++) 
	{
		joints[i].index=i;
		joints[i].tracked = false;
	}

}

void NuitrackTrackerFilt::updateFilteredJoint(int index)
{
	
	mut.lock();
	
	joints[index].position(0,0) = kalmanLinFilters[index].x(0,0);
	joints[index].position(1,0) = kalmanLinFilters[index].x(1,0);
	joints[index].position(2,0) = kalmanLinFilters[index].x(2,0);
	
	joints[index].linearVel(0,0) = kalmanLinFilters[index].x(3,0);
	joints[index].linearVel(1,0) = kalmanLinFilters[index].x(4,0);
	joints[index].linearVel(2,0) = kalmanLinFilters[index].x(5,0);

	joints[index].linearAcc(0,0) = kalmanLinFilters[index].x(6,0);
	joints[index].linearAcc(1,0) = kalmanLinFilters[index].x(7,0);
	joints[index].linearAcc(2,0) = kalmanLinFilters[index].x(8,0);


	joints[index].quaternion(0,0) = kalmanAngFilters[index].x(0,0);
	joints[index].quaternion(1,0) = kalmanAngFilters[index].x(1,0);
	joints[index].quaternion(2,0) = kalmanAngFilters[index].x(2,0);
	joints[index].quaternion(3,0) = kalmanAngFilters[index].x(3,0);

	joints[index].angularVel(0,0) = kalmanAngFilters[index].x(4,0);
	joints[index].angularVel(1,0) = kalmanAngFilters[index].x(5,0);
	joints[index].angularVel(2,0) = kalmanAngFilters[index].x(6,0);

	joints[index].angularAcc(0,0) = kalmanAngFilters[index].x(7,0);
	joints[index].angularAcc(1,0) = kalmanAngFilters[index].x(8,0);
	joints[index].angularAcc(2,0) = kalmanAngFilters[index].x(9,0);

	joints[index].linMeas(0,0) = kalmanLinFilters[index].lastY(0,0);
	joints[index].linMeas(1,0) = kalmanLinFilters[index].lastY(1,0);
	joints[index].linMeas(2,0) = kalmanLinFilters[index].lastY(2,0);

	joints[index].angMeas(0,0) = kalmanAngFilters[index].lastY(0,0);
	joints[index].angMeas(1,0) = kalmanAngFilters[index].lastY(1,0);
	joints[index].angMeas(2,0) = kalmanAngFilters[index].lastY(2,0);
	joints[index].angMeas(3,0) = kalmanAngFilters[index].lastY(3,0);

	joints[index].linT = kalmanLinFilters[index].lastSec;
	joints[index].angT = kalmanAngFilters[index].lastSec;

	mut.unlock();
		
}











#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <cubemos/engine.h>
#include <cubemos/skeleton_tracking.h>

#include "s/s.h"
#include "ros/ros.h"
#include "cubemos_ros/cubemos_body.h"
#include "cubemos_ros/cubemos_ros.hpp"



using CUBEMOS_SKEL_Buffer_Ptr = std::unique_ptr<CM_SKEL_Buffer, void (*)(CM_SKEL_Buffer*)>;

// static const cv::Scalar skeletonColor = cv::Scalar(100, 254, 213);
// static const cv::Scalar jointColor = cv::Scalar(222, 55, 22);

// struct cmPoint {
//     float color_pixel[2];
//     float point3d[3];
//     std::string to_string() const
//     {
//         char buffer[100];
//         int cx = snprintf(buffer, 100, "(%.2f, %.2f, %.2f)", point3d[0], point3d[1], point3d[2]);
//         return std::string(buffer);
//     }
// };

CubemosROS::cmPoint CubemosROS::get_skeleton_point_3d(rs2::depth_frame const& depthFrame, int x, int y)
{
    // Get the distance at the given pixel
    auto distance = depthFrame.get_distance(x, y);

    cmPoint point;
    point.color_pixel[0] = static_cast<float>(x);
    point.color_pixel[1] = static_cast<float>(y);

    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = depthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(point.point3d, &intr, point.color_pixel, distance);

    return point;
}

// cmPoint get_skeleton_point_3d(rs2::depth_frame const& depthFrame, int x, int y)
// {
//     // Get the distance at the given pixel
//     auto distance = depthFrame.get_distance(x, y);

//     cmPoint point;
//     point.color_pixel[0] = static_cast<float>(x);
//     point.color_pixel[1] = static_cast<float>(y);

//     // Deproject from pixel to point in 3D
//     rs2_intrinsics intr = depthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
//     rs2_deproject_pixel_to_point(point.point3d, &intr, point.color_pixel, distance);

//     return point;
// }

// CUBEMOS_SKEL_Buffer_Ptr create_skel_buffer()
// {
//     return CUBEMOS_SKEL_Buffer_Ptr(new CM_SKEL_Buffer(), [](CM_SKEL_Buffer* pb) {
//         cm_skel_release_buffer(pb);
//         delete pb;
//     });
// }

CubemosROS::CUBEMOS_SKEL_Buffer_Ptr CubemosROS::create_skel_buffer()
{
    return CUBEMOS_SKEL_Buffer_Ptr(new CM_SKEL_Buffer(), [](CM_SKEL_Buffer* pb) {
        cm_skel_release_buffer(pb);
        delete pb;
    });
}

/*
Render skeletons and tracking ids on top of the color image
*/
inline void CubemosROS::renderSkeletons(const CM_SKEL_Buffer* skeletons_buffer, rs2::depth_frame const& depth_frame, cv::Mat& image)
{
    CV_Assert(image.type() == CV_8UC3);
    const cv::Point2f absentKeypoint(-1.0f, -1.0f);

    const std::vector<std::pair<int, int>> limbKeypointsIds = { { 1, 2 },   { 1, 5 },   { 2, 3 }, { 3, 4 },  { 5, 6 },
                                                                { 6, 7 },   { 1, 8 },   { 8, 9 }, { 9, 10 }, { 1, 11 },
                                                                { 11, 12 }, { 12, 13 }, { 1, 0 }, { 0, 14 }, { 14, 16 },
                                                                { 0, 15 },  { 15, 17 } };

    for (int i = 0; i < skeletons_buffer->numSkeletons; i++) {
        CV_Assert(skeletons_buffer->skeletons[i].numKeyPoints == 18);

        int id = skeletons_buffer->skeletons[i].id;
        cv::Point2f keyPointHead(skeletons_buffer->skeletons[i].keypoints_coord_x[0],
                                 skeletons_buffer->skeletons[i].keypoints_coord_y[0]);

        for (size_t keypointIdx = 0; keypointIdx < skeletons_buffer->skeletons[i].numKeyPoints; keypointIdx++) {
            const cv::Point2f keyPoint(skeletons_buffer->skeletons[i].keypoints_coord_x[keypointIdx],
                                       skeletons_buffer->skeletons[i].keypoints_coord_y[keypointIdx]);
            if (keyPoint != absentKeypoint) {
                    // get the 3d point and render it on the joints
                cmPoint point3d =
                  get_skeleton_point_3d(depth_frame, static_cast<int>(keyPoint.x), static_cast<int>(keyPoint.y));
                skeleton[keypointIdx].x = point3d.point3d[0];
                skeleton[keypointIdx].y = point3d.point3d[1];
                skeleton[keypointIdx].z = point3d.point3d[2];
			// std::cout << "JOINT: " << keypointIdx << " " << "{" << skeleton[keypointIdx].x << ", " << skeleton[keypointIdx].y << ", " << skeleton[keypointIdx].z << "}" << std::endl;
            }
        }
		// std::cout << skeletons_buffer->skeletons[i].numKeyPoints << std::endl;
		

        // for (const auto& limbKeypointsId : limbKeypointsIds) {
        //     const cv::Point2f keyPointFirst(skeletons_buffer->skeletons[i].keypoints_coord_x[limbKeypointsId.first],
        //                                     skeletons_buffer->skeletons[i].keypoints_coord_y[limbKeypointsId.first]);

        //     const cv::Point2f keyPointSecond(skeletons_buffer->skeletons[i].keypoints_coord_x[limbKeypointsId.second],
        //                                      skeletons_buffer->skeletons[i].keypoints_coord_y[limbKeypointsId.second]);

        //     if (keyPointFirst == absentKeypoint || keyPointSecond == absentKeypoint) {
        //         continue;
        //     }

        //     cv::line(image, keyPointFirst, keyPointSecond, skeletonColor, 2, cv::LINE_AA);
        // }
        // for (size_t keypointIdx = 0; keypointIdx < skeletons_buffer->skeletons[i].numKeyPoints; keypointIdx++) {
        //     const cv::Point2f keyPoint(skeletons_buffer->skeletons[i].keypoints_coord_x[keypointIdx],
        //                                skeletons_buffer->skeletons[i].keypoints_coord_y[keypointIdx]);
        //     if (keyPoint != absentKeypoint) {
        //         // found a valid keypoint and displaying the skeleton tracking id next to it
        //         cv::putText(image,
        //                     (std::to_string(id)),
        //                     cv::Point2f(keyPoint.x, keyPoint.y - 20),
        //                     cv::FONT_HERSHEY_COMPLEX,
        //                     1,
        //                     skeletonColor);
        //         break;
        //     }
        // }
    }
}

// inline void renderSkeletons(const CM_SKEL_Buffer* skeletons_buffer, rs2::depth_frame const& depth_frame, cv::Mat& image)
// {
//     CV_Assert(image.type() == CV_8UC3);
//     const cv::Point2f absentKeypoint(-1.0f, -1.0f);

//     const std::vector<std::pair<int, int>> limbKeypointsIds = { { 1, 2 },   { 1, 5 },   { 2, 3 }, { 3, 4 },  { 5, 6 },
//                                                                 { 6, 7 },   { 1, 8 },   { 8, 9 }, { 9, 10 }, { 1, 11 },
//                                                                 { 11, 12 }, { 12, 13 }, { 1, 0 }, { 0, 14 }, { 14, 16 },
//                                                                 { 0, 15 },  { 15, 17 } };

//     for (int i = 0; i < skeletons_buffer->numSkeletons; i++) {
//         CV_Assert(skeletons_buffer->skeletons[i].numKeyPoints == 18);

//         int id = skeletons_buffer->skeletons[i].id;
//         cv::Point2f keyPointHead(skeletons_buffer->skeletons[i].keypoints_coord_x[0],
//                                  skeletons_buffer->skeletons[i].keypoints_coord_y[0]);

//         for (size_t keypointIdx = 0; keypointIdx < skeletons_buffer->skeletons[i].numKeyPoints; keypointIdx++) {
//             const cv::Point2f keyPoint(skeletons_buffer->skeletons[i].keypoints_coord_x[keypointIdx],
//                                        skeletons_buffer->skeletons[i].keypoints_coord_y[keypointIdx]);
//             if (keyPoint != absentKeypoint) {
//                     // get the 3d point and render it on the joints
//                 cmPoint point3d =
//                   get_skeleton_point_3d(depth_frame, static_cast<int>(keyPoint.x), static_cast<int>(keyPoint.y));
// 			// std::cout << "{" << point3d.point3d[0] << ", " << point3d.point3d[1] << ", " << point3d.point3d[2] << "}" << std::endl;
//             }
//         }
// 		// std::cout << skeletons_buffer->skeletons[i].numKeyPoints << std::endl;
		

//         // for (const auto& limbKeypointsId : limbKeypointsIds) {
//         //     const cv::Point2f keyPointFirst(skeletons_buffer->skeletons[i].keypoints_coord_x[limbKeypointsId.first],
//         //                                     skeletons_buffer->skeletons[i].keypoints_coord_y[limbKeypointsId.first]);

//         //     const cv::Point2f keyPointSecond(skeletons_buffer->skeletons[i].keypoints_coord_x[limbKeypointsId.second],
//         //                                      skeletons_buffer->skeletons[i].keypoints_coord_y[limbKeypointsId.second]);

//         //     if (keyPointFirst == absentKeypoint || keyPointSecond == absentKeypoint) {
//         //         continue;
//         //     }

//         //     cv::line(image, keyPointFirst, keyPointSecond, skeletonColor, 2, cv::LINE_AA);
//         // }
//         // for (size_t keypointIdx = 0; keypointIdx < skeletons_buffer->skeletons[i].numKeyPoints; keypointIdx++) {
//         //     const cv::Point2f keyPoint(skeletons_buffer->skeletons[i].keypoints_coord_x[keypointIdx],
//         //                                skeletons_buffer->skeletons[i].keypoints_coord_y[keypointIdx]);
//         //     if (keyPoint != absentKeypoint) {
//         //         // found a valid keypoint and displaying the skeleton tracking id next to it
//         //         cv::putText(image,
//         //                     (std::to_string(id)),
//         //                     cv::Point2f(keyPoint.x, keyPoint.y - 20),
//         //                     cv::FONT_HERSHEY_COMPLEX,
//         //                     1,
//         //                     skeletonColor);
//         //         break;
//         //     }
//         // }
//     }
// }

CubemosROS::CubemosROS()
{
	initVariables();

	initFilteredJoints();

	cout << "initializing filters.." << endl;
	
	for (int i=0;i<joints.size();i++)
	{
		initMediumFilter(i);
	}

	cout << "initializing cubemos..." << endl;
	// try
	// {
		initCubemos();
	// }
	// catch(const Exception& e)
	// {
	// 	cerr << "Can not initialize the tracker (ExceptionType: " << e.type() << ")" << endl;
	// 	return;
	// }
	
	cout << "init ok" << endl;	
}

void CubemosROS::initVariables()
{
	updating = false;
	configPath = "";
	//configPath = "/home/liralab/nuitrackCustomConfig/nuitrack.config";
	FilteredJoint nJ;
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

    skeleton.resize(18);

	xLin = MatrixXd::Zero(9,1);
	Qlin = (VectorXd(9) << 0,0,0,1e-6,1e-6,1e-6,1e-2,1e-2,1e-2).finished().asDiagonal();
	Rlin = (VectorXd(3) << 1e-9,1e-9,1e-9).finished().asDiagonal();
	Plin = (VectorXd(9) << 1,1,1,1,1,1,1,1,1).finished().asDiagonal();
	yLin = MatrixXd::Zero(3,1);

	xAng = MatrixXd::Zero(10,1);
	xAng(0,0) = 1.0;
	Qang = (VectorXd(10) << 0,0,0,0,1e-6,1e-6,1e-6,1e-3,1e-3,1e-3).finished().asDiagonal();
	Rang = (VectorXd(4) << 1e-10,1e-10,1e-10,1e-10).finished().asDiagonal();
	Pang = (VectorXd(10) << 1,1,1,1,1,1,1,1,1,1).finished().asDiagonal();
	yAng = MatrixXd::Zero(4,1);
	rot  << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	quat = Quaterniond(0.0,0.0,0.0,0.0);
	quatOld = Quaterniond(0.0,0.0,0.0,0.0);	

	//DA SISTEMARE I NOMI DI FIANCO
	trackedJoints.push_back(0); //nose
	trackedJoints.push_back(1); //nose
	trackedJoints.push_back(2); //neck
	trackedJoints.push_back(3); //torso	
	trackedJoints.push_back(4); //waist	
	trackedJoints.push_back(5); //left collar	
	trackedJoints.push_back(6); //left shoulder
	trackedJoints.push_back(7); //left elbow
	trackedJoints.push_back(8); //left wrist	
	trackedJoints.push_back(9); //left hand	
	trackedJoints.push_back(10); //left fingertip
	trackedJoints.push_back(11); //right collar
	trackedJoints.push_back(12); //right shoulder
	trackedJoints.push_back(13); //right elbow
	trackedJoints.push_back(14); //right wrist
	trackedJoints.push_back(15); //right hand
	trackedJoints.push_back(16); //right fingertip
	trackedJoints.push_back(17); //left hip
	
	chronoStart = chrono::system_clock::now();

	
}

void CubemosROS::initFilteredJoints()
{
	for (int i=0;i<joints.size();i++) 
	{
		joints[i].index=i;
		joints[i].tracked = false;
	}

}

void CubemosROS::initMediumFilter(int index)
{
	mfilt = MediumFilt(10);
	kang = KalmanRot(xAng,Pang,Qang,Rang,index);
	
	if (MediumFilters.size()<index+1) 
	{
		MediumFilters.push_back(mfilt);
		kalmanAngFilters.push_back(kang);
	}
	else
	{
		MediumFilters[index] = mfilt;
		kalmanAngFilters[index] = kang;
	}

}

void CubemosROS::initFilter(int index)
{
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

int CubemosROS::initCubemos()
{

	updating = true;

	// set up the intel realsense pipeline
	ROS_INFO("realsense init.. ");

    if (ctx.query_devices().size() == 0) {
		EXIT_PROGRAM("No realsense device connected.");       
    }

    cfg.enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 1280, 720, RS2_FORMAT_ANY, 30);
    align_to_color = new rs2::align(RS2_STREAM_DEPTH);
	
	rs2::pipeline_profile profile;
	try {
		profile = pipe.start(cfg);
	}
	catch (std::exception ex)
	{
		EXIT_PROGRAM(std::string("Exception encountered starting the RealSense pipeline: ") + ex.what());		
	}
    auto sensor = profile.get_device().first<rs2::depth_sensor>();

    auto range = sensor.get_option_range(RS2_OPTION_VISUAL_PRESET);
    for (auto i = range.min; i < range.max; i += range.step)
        if (std::string(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, i)) == "High Density")
            sensor.set_option(RS2_OPTION_VISUAL_PRESET, i);

	ROS_INFO("cubemos init...");
    CM_TargetComputeDevice enInferenceMode = CM_TargetComputeDevice::CM_CPU;


    // set up the cubemos skeleton tracking api pipeline
    handle = nullptr;
    // Output all messages with severity level INFO or higher to the console and to a file
    cm_initialise_logging(CM_LogLevel::CM_LL_INFO, true, default_log_dir().c_str());

    retCode = cm_skel_create_handle(&handle, default_license_dir().c_str());
    CHECK_HANDLE_CREATION(retCode);

    std::string modelName = default_model_dir();
    if (enInferenceMode == CM_TargetComputeDevice::CM_CPU) {
        modelName += std::string("/fp32/skeleton-tracking.cubemos");
    }
    else {
        modelName += std::string("/fp16/skeleton-tracking.cubemos");
    }
    retCode = cm_skel_load_model(handle, enInferenceMode, modelName.c_str());
    if (retCode != CM_SUCCESS) {
        //EXIT_PROGRAM("Model loading failed.");
    }

    // std::string cvWindowName = "cubemos: skeleton tracking with intel realsense camera C/C++";
    //cv::namedWindow(cvWindowName, cv::WINDOW_NORMAL);
    //cv::setWindowProperty(cvWindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

     // height of the image with which the DNN model will run inference

    // cache the first inference to get started with tracking
    // let some empty frames to run
    for (int k = 0; k < 30; k++) {
        rs2::frameset data = pipe.wait_for_frames();
        rs2::frame colorFrame = data.get_color_frame();
        rs2::frame depthFrame = data.get_depth_frame();
        capturedFrame = cv::Mat(
          cv::Size(colorFrame.as<rs2::video_frame>().get_width(), colorFrame.as<rs2::video_frame>().get_height()),
          CV_8UC3,
          (void*)colorFrame.get_data(),
          cv::Mat::AUTO_STEP);
    }

    CM_Image imageLast = {
        capturedFrame.data, CM_UINT8, capturedFrame.cols, capturedFrame.rows, capturedFrame.channels(),
        (int)capturedFrame.step[0], CM_HWC
    };

    skeletonsPresent = new CUBEMOS_SKEL_Buffer_Ptr(create_skel_buffer());
    skeletonsLast = new CUBEMOS_SKEL_Buffer_Ptr(create_skel_buffer());
	int nTimeoutMs = 1000;

    // Get the skeleton keypoints for the first frame
    CM_ReturnCode retCodeFirstFrame = cm_skel_estimate_keypoints(handle, &imageLast, nHeight, (*skeletonsLast).get());

    // continue to loop through acquisition and display until the escape key is hit
    frameCount = 0;
    fpsTest = "Frame rate: ";

    // start measuring the time taken for execution
    startTime = std::chrono::system_clock::now();

    cout << "starting cubemos thread..." << endl;
    thread updatingThread(bind(&CubemosROS::update,this));
    updatingThread.detach();

}

void CubemosROS::update()
{
	while(updating)
	{

		//UPDATE DELLO SKELETON TRACKING E DEI FILTRI
		// capture image
        rs2::frameset data = pipe.wait_for_frames();
        data = align_to_color->process(data);

        rs2::frame colorFrame = data.get_color_frame();
        rs2::frame depthFrame = data.get_depth_frame();

        capturedFrame = cv::Mat(
          cv::Size(colorFrame.as<rs2::video_frame>().get_width(), colorFrame.as<rs2::video_frame>().get_height()),
          CV_8UC3,
          (void*)colorFrame.get_data(),
          cv::Mat::AUTO_STEP);

        // exit the loop if the captured frame is empty
        if (capturedFrame.empty()) {
            std::cerr << "No new frame could be captured using the input source. Exiting the loop." << std::endl;
            break;
        }

        CM_Image imagePresent = {
            capturedFrame.data,         CM_UINT8, capturedFrame.cols, capturedFrame.rows, capturedFrame.channels(),
            (int)capturedFrame.step[0], CM_HWC
        };

        // Run Skeleton Tracking and display the results
        retCode = cm_skel_estimate_keypoints(handle, &imagePresent, nHeight, (*skeletonsPresent).get());

        // track the skeletons in case of successful skeleton estimation
        if (retCode == CM_SUCCESS) {
            if ((*skeletonsPresent)->numSkeletons > 0) {
                // Assign tracking ids to the skeletons in the present frame
                cm_skel_update_tracking_id(handle, (*skeletonsLast).get(), (*skeletonsPresent).get());
                // Render skeleton overlays with tracking ids
                renderSkeletons((*skeletonsPresent).get(), depthFrame, capturedFrame);
                // // Set the present frame as last one to track the next frame
                (*skeletonsLast).swap(*skeletonsPresent);
                // // Free memory of the latest frame
                cm_skel_release_buffer((*skeletonsPresent).get());
            }
        }

        frameCount++;
        if (frameCount % 25 == 0) {
            auto timePassed =
              std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime)
                .count();
            auto fps = 25000.0 / timePassed;

            fpsTest = "Frame rate: " + std::to_string(fps) + " FPS";
			std::cout << fpsTest << std::endl;
            startTime = std::chrono::system_clock::now();
        }
		
        // cv::putText(capturedFrame, fpsTest, cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, skeletonColor);

        // cv::imshow(cvWindowName, capturedFrame);

        for (int i = 0; i < skeleton.size(); i++)
        {
            if((abs(skeleton[i].x) + abs(skeleton[i].y) + abs(skeleton[i].z)) > 0.001)
            {
                updateFilter(i);
                joints[i].tracked = true;
            }
            else
            {
                joints[i].tracked = false;
                initMediumFilter(i);
                // initFilter(i);
            }

        }

		for (int i=0;i<joints.size();i++)
		{
			updateFilteredJoint(i);
		}
	}
}

void CubemosROS::updateFilter(int index)
{

	// skip update if index is not in trackedJoints vector
	
	if (find(trackedJoints.begin(),trackedJoints.end(),index) == trackedJoints.end()) return;

	

	// position measurements	
	
	yLin(0,0) = skeleton[index].x; 
	// yLin(1,0) = -skeleton[index].y;  // left-handed correction
	yLin(1,0) = skeleton[index].y;
	yLin(2,0) = skeleton[index].z;


	// // orientation measurements	

	// rot(0,0) = currentSkeleton.joints[index].orient.matrix[0];
	// rot(0,1) = currentSkeleton.joints[index].orient.matrix[1];
	// rot(0,2) = currentSkeleton.joints[index].orient.matrix[2];
				
	// rot(1,0) = currentSkeleton.joints[index].orient.matrix[3];
	// rot(1,1) = currentSkeleton.joints[index].orient.matrix[4];
	// rot(1,2) = currentSkeleton.joints[index].orient.matrix[5];

	// rot(2,0) = currentSkeleton.joints[index].orient.matrix[6];
	// rot(2,1) = currentSkeleton.joints[index].orient.matrix[7];
	// rot(2,2) = currentSkeleton.joints[index].orient.matrix[8];

	rot(0,0) = 1;
	rot(0,1) = 0;
	rot(0,2) = 0;
				
	rot(1,0) = 0;
	rot(1,1) = 1;
	rot(1,2) = 0;

	rot(2,0) = 0;
	rot(2,1) = 0;
	rot(2,2) = 1;

	
	
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
	

	MediumFilters[index].update(yLin);
	// kalmanLinFilters[index].update(yLin);	
	kalmanAngFilters[index].update(yAng);	

}

void CubemosROS::updateFilteredJoint(int index)
{
	
	mut.lock();
	
	// joints[index].position(0,0) = kalmanLinFilters[index].x(0,0);
	// joints[index].position(1,0) = kalmanLinFilters[index].x(1,0);
	// joints[index].position(2,0) = kalmanLinFilters[index].x(2,0);
	
	// joints[index].linearVel(0,0) = kalmanLinFilters[index].x(3,0);
	// joints[index].linearVel(1,0) = kalmanLinFilters[index].x(4,0);
	// joints[index].linearVel(2,0) = kalmanLinFilters[index].x(5,0);

	// joints[index].linearAcc(0,0) = kalmanLinFilters[index].x(6,0);
	// joints[index].linearAcc(1,0) = kalmanLinFilters[index].x(7,0);
	// joints[index].linearAcc(2,0) = kalmanLinFilters[index].x(8,0);

	joints[index].position(0,0) = MediumFilters[index].x(0,0);
	joints[index].position(1,0) = MediumFilters[index].x(1,0);
	joints[index].position(2,0) = MediumFilters[index].x(2,0);
	
	joints[index].linearVel(0,0) = MediumFilters[index].x(3,0);
	joints[index].linearVel(1,0) = MediumFilters[index].x(4,0);
	joints[index].linearVel(2,0) = MediumFilters[index].x(5,0);
	
	joints[index].linearAcc(0,0) = 0;
	joints[index].linearAcc(1,0) = 0;
	joints[index].linearAcc(2,0) = 0;


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

	// joints[index].linMeas(0,0) = kalmanLinFilters[index].lastY(0,0);
	// joints[index].linMeas(1,0) = kalmanLinFilters[index].lastY(1,0);
	// joints[index].linMeas(2,0) = kalmanLinFilters[index].lastY(2,0);

	joints[index].linMeas(0,0) = 0;
	joints[index].linMeas(1,0) = 0;
	joints[index].linMeas(2,0) = 0;

	joints[index].angMeas(0,0) = kalmanAngFilters[index].lastY(0,0);
	joints[index].angMeas(1,0) = kalmanAngFilters[index].lastY(1,0);
	joints[index].angMeas(2,0) = kalmanAngFilters[index].lastY(2,0);
	joints[index].angMeas(3,0) = kalmanAngFilters[index].lastY(3,0);

	// joints[index].linT = kalmanLinFilters[index].lastSec;
	joints[index].linT = 0;
	joints[index].angT = kalmanAngFilters[index].lastSec;

	mut.unlock();
		
}

vector<FilteredJoint> CubemosROS::getFilteredJoints()
{	
	return joints;
}

void mapJoint(cubemos_ros::cubemos_body* msg, FilteredJoint joint) 
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
	// if (joint.index == 0)
	// {
	// 	cout << "neck position x " << joint.position(0,0)<<endl;
	// 	cout << "neck vel x " << joint.linearVel(0,0)<<endl;
	// 	cout << "neck acc x " << joint.linearAcc(0,0)<<endl; 
	// }


	
	// joint map
	switch (joint.index)
	{
		case (0): // none
			msg->nose = tmp;
			msg->nose_vel = tmpTwist;
			msg->nose_acc = tmpAcc;
		break;
		case (1): // head
			msg->torso = tmp;
			msg->torso_vel = tmpTwist;
			msg->torso_acc = tmpAcc;
		break;
		case (2): // neck
			msg->right_shoulder = tmp;
			msg->right_shoulder_vel = tmpTwist;
			msg->right_shoulder_acc = tmpAcc;
		break;
		case (3): // torso			
			msg->right_elbow = tmp;			
			msg->right_elbow_vel = tmpTwist;
			msg->right_elbow_acc = tmpAcc;
		break;
		case (4): // waist
			msg->right_hand = tmp;			
			msg->right_hand_vel = tmpTwist;
			msg->right_hand_acc = tmpAcc;
		break;
		case (5): // neck
			msg->left_shoulder = tmp;
			msg->left_shoulder_vel = tmpTwist;
			msg->left_shoulder_acc = tmpAcc;
		break;
		case (6): // torso			
			msg->left_elbow = tmp;			
			msg->left_elbow_vel = tmpTwist;
			msg->left_elbow_acc = tmpAcc;
		break;
		case (7): // waist
			msg->left_hand = tmp;			
			msg->left_hand_vel = tmpTwist;
			msg->left_hand_acc = tmpAcc;
        break;
		case (8): // left wrist
            msg->right_thigh = tmp;
            msg->right_thigh_vel = tmpTwist;
            msg->right_thigh_acc = tmpAcc;	
		break;
		case (9): // left wrist
            msg->right_knee = tmp;
            msg->right_knee_vel = tmpTwist;
            msg->right_knee_acc = tmpAcc;	
		break;
		case (10): // left wrist
            msg->right_foot = tmp;
            msg->right_foot_vel = tmpTwist;
            msg->right_foot_acc = tmpAcc;	
		break;
		case (11): // left wrist
            msg->left_thigh = tmp;
            msg->left_thigh_vel = tmpTwist;
            msg->left_thigh_acc = tmpAcc;	
		break;
		case (12): // left wrist
            msg->left_knee = tmp;
            msg->left_knee_vel = tmpTwist;
            msg->left_knee_acc = tmpAcc;	
		break;
		case (13): // left wrist
            msg->left_foot = tmp;
            msg->left_foot_vel = tmpTwist;
            msg->left_foot_acc = tmpAcc;	
		break;
		case (14): // left wrist
            msg->right_eye = tmp;
            msg->right_eye_vel = tmpTwist;
            msg->right_eye_acc = tmpAcc;	
		break;
		case (15): // left wrist
            msg->left_eye = tmp;
            msg->left_eye_vel = tmpTwist;
            msg->left_eye_acc = tmpAcc;	
		break;
		case (16): // left wrist
            msg->right_ear = tmp;
            msg->right_ear_vel = tmpTwist;
            msg->right_ear_acc = tmpAcc;	
		break;
		case (17): // left wrist
            msg->left_ear = tmp;
            msg->left_ear_vel = tmpTwist;
            msg->left_ear_acc = tmpAcc;	
		break;	
	}

}

int main(int argc, char **argv)
{
	ROS_INFO("node init...");
	ros::init(argc, argv, "cubemos_tracker");
	ros::NodeHandle nh;
	ros::Publisher skeleton_pub = nh.advertise<cubemos_ros::cubemos_body>("skeleton_topic", 1); 	
	ros::Rate loop_rate(60);

	CubemosROS CubemosRos;
	vector<FilteredJoint> nJoints;

	ROS_INFO("Tracking loop starting.. ");
	while (ros::ok())
	{
		cubemos_ros::cubemos_body msg;
		nJoints = CubemosRos.getFilteredJoints();


		
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

    // while (ros::ok()) {
        // capture image
        // rs2::frameset data = pipe.wait_for_frames();
        // data = align_to_color.process(data);

        // rs2::frame colorFrame = data.get_color_frame();
        // rs2::frame depthFrame = data.get_depth_frame();

        // capturedFrame = cv::Mat(
        //   cv::Size(colorFrame.as<rs2::video_frame>().get_width(), colorFrame.as<rs2::video_frame>().get_height()),
        //   CV_8UC3,
        //   (void*)colorFrame.get_data(),
        //   cv::Mat::AUTO_STEP);

        // // exit the loop if the captured frame is empty
        // if (capturedFrame.empty()) {
        //     std::cerr << "No new frame could be captured using the input source. Exiting the loop." << std::endl;
        //     break;
        // }

        // CM_Image imagePresent = {
        //     capturedFrame.data,         CM_UINT8, capturedFrame.cols, capturedFrame.rows, capturedFrame.channels(),
        //     (int)capturedFrame.step[0], CM_HWC
        // };

        // // Run Skeleton Tracking and display the results
        // retCode = cm_skel_estimate_keypoints(handle, &imagePresent, nHeight, skeletonsPresent.get());

        // // track the skeletons in case of successful skeleton estimation
        // if (retCode == CM_SUCCESS) {
        //     if (skeletonsPresent->numSkeletons > 0) {
        //         // Assign tracking ids to the skeletons in the present frame
        //         cm_skel_update_tracking_id(handle, skeletonsLast.get(), skeletonsPresent.get());
        //         // Render skeleton overlays with tracking ids
        //         renderSkeletons(skeletonsPresent.get(), depthFrame, capturedFrame);
        //         // Set the present frame as last one to track the next frame
        //         skeletonsLast.swap(skeletonsPresent);
        //         // Free memory of the latest frame
        //         cm_skel_release_buffer(skeletonsPresent.get());
        //     }
        // }

        // frameCount++;
        // if (frameCount % 25 == 0) {
        //     auto timePassed =
        //       std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime)
        //         .count();
        //     auto fps = 25000.0 / timePassed;

        //     fpsTest = "Frame rate: " + std::to_string(fps) + " FPS";
		// 	std::cout << fpsTest << std::endl;
        //     startTime = std::chrono::system_clock::now();
        // }
        // cv::putText(capturedFrame, fpsTest, cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX, 1, skeletonColor);

        // cv::imshow(cvWindowName, capturedFrame);
    // }

    // release the memory which is managed by the cubemos framework
    // cm_skel_destroy_handle(&handle);
    return 0;
}

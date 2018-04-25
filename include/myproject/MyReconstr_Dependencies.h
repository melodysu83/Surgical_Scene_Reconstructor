#ifndef MYRECONSTR_DEPENDENCIES_H
#define MYRECONSTR_DEPENDENCIES_H

// dependencies on c++ libraries
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <iostream>
#include <time.h> 
#include <stdio.h>
#include <iomanip>
#include <pthread.h>
#include <termios.h>
#include <signal.h>
#include <queue>
#include <unistd.h>
#include <cmath>

// dependencies on ros packages
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

// eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

// dependencies on opencv libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/opencv.hpp>

#define ROS_PARAMETER_NAME0 "/Surgical_Scene_Reconstructor/CAMERA_COUNT"
#define ROS_PARAMETER_NAME1 "/Surgical_Scene_Reconstructor/IMG_FOLDER"
#define ROS_PARAMETER_NAME2 "/Surgical_Scene_Reconstructor/TME_STAMP"
#define ROS_PARAMETER_NAME3 "/Surgical_Scene_Reconstructor/CAM_POSE"
#define ROS_PARAMETER_NAME4 "/Surgical_Scene_Reconstructor/CAM_CALI"

#define ROS_TOPIC_PUBLISH_NAME1 "/Surgical_Scene_Reconstructor/2D_images"
#define ROS_TOPIC_PUBLISH_NAME2 "/Surgical_Scene_Reconstructor/3D_model"

#define OPENCV_IMSHOW_WINDOW_NAME "My Image"
#define OPENCV_IMSHOW_WINDOW_SCALE 1.7

#define CONSOLE_LOOP_RATE   10     // in Hz
#define IO_LOOP_RATE        30     // in Hz

#define CAM_POSE_SIZE       16     // number of entries of T matrix
#define TIME_STEP_SIZE	    0.01   // in sec
#define TIME_STAMP_COL	    2      // the column number in timestamp file that's in use

#define IMAGE_H		    480    // image height
#define IMAGE_W		    640    // image width
#define IMAGE_FILE_DIGITS   5      // number of digits in the image files names

#define DEFAULT_IMAGE_FUNC_SIGMA 3.0

// namespace declaration
using namespace std;
using namespace cv;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

// enum declaration
enum IMAGE_STATUS_LIST{
	IMAGE_EMPTY,       // 0
	IMAGE_START_LOADING,     // 1
	IMAGE_DONE_LOADING,      // 2
	IMAGE_START_PROCESSING,  // 3
	IMAGE_DONE_PROCESSING,   // 4
	IMAGE_START_PUBLISHING,  // 5
	IMAGE_DONE_PUBLISHING    // 6
};

enum MODEL_STATUS_LIST{
	MODEL_EMPTY,       	 // 0
	MODEL_START_PROCESSING,  // 1
	MODEL_DONE_PROCESSING,   // 2
	MODEL_START_PUBLISHING,  // 3
	MODEL_DONE_PUBLISHING    // 4
};

enum SYSTEM_STATUS_LIST{
	SYSTEM_JUST_STARTING,       	// 0
	SYSTEM_SHOW_MENU,               // 1
	SYSTEM_PENDING_USER_SELECTION,  // 2
	SYSTEM_DATA_DISPLAY_MODE,	// 3
	SYSTEM_RECONSTR_QUIET_MODE, 	// 4
	SYSTEM_RECONSTR_DEBUG_MODE,	// 5
	SYSTEM_DATA_ENDING,		// 6
	SYSTEM_EXIT_ALL,                // 7
	SYSTEM_SHOW_FEATURE_MENU,	// 8
	SYSTEM_PENDING_FEATURE_CHOICE   // 9
	
};

enum FEATURE_ALGO_LIST{
	FAST_ALGO,	// 0 : Suggested by OpenCV for real time application (https://docs.opencv.org/3.1.0/df/d0c/tutorial_py_fast.html)

	SURF_ALGO,	// 1 : Used in COSLAM (https://github.com/danping/CoSLAM/blob/master/src/app/SL_InitMap.cpp)
			//     which claims to be fast than sift? (https://stackoverflow.com/questions/11172408/surf-vs-sift-is-surf-really-faster)

	ORB_ALGO,	// 2 : Used in mapping3d (https://github.com/Tetragramm/opencv_contrib/blob/master/modules/mapping3d/samples/computeMapping3d.cpp)
			//     claimed to be orientation invarient than FAST (https://docs.opencv.org/3.1.0/d1/d89/tutorial_py_orb.html)
};

#define NUM_OF_FEATURE_ALGO 3
#define DEFAULT_FEATURE_ALGO FAST_ALGO

// for FAST_ALGO
#define DEFAULT_FEATURE_PARAM_INTENSITY_THRES 20
#define DEFAULT_FEATURE_PARAM_NON_MAX_SUPPRE true

// for SURF_ALGO
#define DEFAULT_FEATURE_PARAM_HESSIAN_THRES 300
#define DEFAULT_FEATURE_PARAM_N_PYRAMIDS 4
#define DEFAULT_FEATURE_PARAM_N_PYRAMID_LAYERS 3
#define DEFAULT_FEATURE_PARAM_DESCRIPTOR_EXTENDED false

// for ORB_ALGO
#define DEFAULT_FEATURE_PARAM_N_FEATURES 2000
#define DEFAULT_FEATURE_PARAM_SCALE 1.2f
#define DEFAULT_FEATURE_PARAM_N_LEVELS 5
#define DEFAULT_FEATURE_PARAM_EDGE_THRES 11

#define NUM_OF_FEATURE_PARAMETERS 10

string FEATURE_ALGO_TO_STRING(FEATURE_ALGO_LIST);
#endif

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

// dependencies on ros packages
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// dependencies on opencv libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv/cv.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

// dependencies on self-created classes
#include "myproject/MyReconstr_Controller.h"
#include "myproject/MyReconstr_Display.h"

// namespace declaration
using namespace std;
using namespace cv;

// type define
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#endif

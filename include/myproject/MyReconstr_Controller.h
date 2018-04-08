#ifndef MYRECONSTR_CONTROLLER_H
#define MYRECONSTR_CONTROLLER_H

#include "myproject/MyReconstr_Dependencies.h"

class MyReconstr_Controller
{
	private:		
		int CAMERA_COUNT;
		int* IMAGE_COUNT;
		string* IMG_FOLDER;
		string* TME_STAMP_FILE;
		string* CAM_POSE_FILE;
		string* CAM_CALI_FILE;
		
		bool NEW_IMAGE_TO_PUB;
		bool NEW_MODEL_TO_PUB;
		IMAGE_STATUS_LIST IMAGE_STATUS;
		MODEL_STATUS_LIST MODEL_STATUS;
		SYSTEM_STATUS_LIST SYSTEM_STATUS;

		PointCloud Model3D;  			//ref: http://wiki.ros.org/pcl_ros
		cv_bridge::CvImagePtr Images2D;

		ros::NodeHandle nh_; 
		ros::Publisher model_pub_;
		image_transport::ImageTransport it_;
		image_transport::Publisher image_pub_;

		MyReconstr_Display CONSOLE;
	public:
		MyReconstr_Controller(int argc, char** argv);
		~MyReconstr_Controller();

		void init_sys();
		void load_ros_param();
		void load_dataset();
		bool init_ros(int, char**);

		void start_thread();
		void join_thread();

		void modelPb();
		void imagePb();
};

#endif

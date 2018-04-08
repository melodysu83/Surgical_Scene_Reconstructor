#ifndef MYRECONSTR_CONTROLLER_H
#define MYRECONSTR_CONTROLLER_H

#include "myproject/MyReconstr_Display.h"
class MyReconstr_Controller
{
	private:		
		string* IMG_FOLDER;
		string* TME_STAMP_FILE;
		string* CAM_POSE_FILE;
		string* CAM_CALI_FILE;

		int CAMERA_COUNT;
		int IMAGE_PUB_COUNT;
		int MODEL_PUB_COUNT;

		bool GOODBYE;
		bool NEW_IMAGE_TO_PUB;
		bool NEW_MODEL_TO_PUB;

		IMAGE_STATUS_LIST IMAGE_STATE;
		MODEL_STATUS_LIST MODEL_STATE;
		SYSTEM_STATUS_LIST SYSTEM_STATE;

		PointCloud<PointXYZRGBNormal> Model3D;  			
		cv_bridge::CvImagePtr Images2D;

		ros::NodeHandle nh_; 
		ros::Publisher model_pub_;
		image_transport::ImageTransport it_;
		image_transport::Publisher image_pub_;

		pthread_t console_thread;	// show system status and takes user choices
		pthread_t io_thread; 		// image input and 3d model output
		pthread_t reconstr_thread;	// do 3D reconstruction

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
		static void sigint_handler(int);

		void *console_process(void);
		void *io_process(void);
		void *reconstr_process(void);
		static void *static_console_process(void*);
		static void *static_io_process(void*);
		static void *static_reconstr_process(void*);

		void modelPb();
		void imagePb();
};

#endif

#ifndef MYRECONSTR_CONTROLLER_H
#define MYRECONSTR_CONTROLLER_H

#include "myproject/MyReconstr_Vision.h"
class MyReconstr_Controller
{
	private:		
		int CAMERA_COUNT;
		int IMAGE_PUB_COUNT;
		int MODEL_PUB_COUNT;
		double SYSTEM_TIME;

		bool GOODBYE;
		bool USER_INPUT_PAUSE;
		bool USER_INPUT_SHOWSTATUS;
		bool NEW_IMAGE_TO_PUB;
		bool NEW_MODEL_TO_PUB;

		IMAGE_STATUS_LIST IMAGE_STATE;
		MODEL_STATUS_LIST MODEL_STATE;
		SYSTEM_STATUS_LIST SYSTEM_STATE;

		vector<cv::Mat> CURRENT_IMAGES;
		vector<vector<double> > CURRENT_CAM_POSES;

		PointCloud<PointXYZRGB> Model3D;  
		PointCloud<PointXYZRGB> Model3D_Last;  				
		cv::Mat Images2D;		
		cv::Mat Images2D_Last;

		ros::NodeHandle nh_; 
		ros::Publisher model_pub_;
		image_transport::ImageTransport it_;
		image_transport::Publisher image_pub_;

		pthread_t console_thread;	// show system status and takes user choices
		pthread_t io_thread; 		// image input and 3d model output
		pthread_t reconstr_thread;	// do 3D reconstruction

		MyReconstr_Display CONSOLE;
		MyReconstr_Storage DATABANK;
		MyReconstr_Vision VISIONTOOL;
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

		void load_currect_images_and_pose();
		void clean_up_and_ready_for_restart();
		void final_result_publish_and_display();

		void modelPb();
		void imagePb();
};

#endif

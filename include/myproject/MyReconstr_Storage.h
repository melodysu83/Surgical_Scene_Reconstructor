#ifndef MYRECONSTR_STORAGE_H
#define MYRECONSTR_STORAGE_H

#include "myproject/MyReconstr_Display.h"
class MyReconstr_Storage
{
	private:		
		int CAMERA_COUNT;
		double SYSTEM_TIME;
		bool DATA_END_FLAG;

		string* IMG_FOLDER;
		string* TME_STAMP_FILE;
		string* CAM_POSE_FILE;
		string* CAM_CALI_FILE;

		vector<vector<vector<double> > > TME_STAMP_DATA;
		vector<vector<vector<double> > > CAM_POSE_DATA;
		vector<cv::Mat> CALI_INTRI_DATA;
		vector<cv::Mat> CALI_DISTO_DATA;

		vector<int> CURRENT_IMAGES_INDEX;

		MyReconstr_Display CONSOLE;
	public:
		MyReconstr_Storage();
		~MyReconstr_Storage();
		void set_camera_count(int);
		void set_ros_param_val(int,vector<string>);
		vector<char*> get_ros_param_name(int);
		string get_img_folder_name(int);
		void load_dataset();
		void verify_loaded_dataset();

		void set_system_time(double);
		bool check_data_ending();
		void reset_data_pointers();
		cv::Mat get_random_image();	       		        	// external call
		vector<cv::Mat> get_random_images();	       		        // internal processing
		vector<cv::Mat> get_current_images();		       		// internal processing
		vector<cv::Mat> get_current_images(double);	       		// external call
		vector<vector<double> > get_current_cam_poses(bool);   		// external call
		vector<vector<double> > get_current_cam_poses(vector<double>); 	// internal processing
		int current_image_index_binary_search(int,int,int);
		int current_image_index_find_closest(int,int);
		int image_index_binary_search(double,int,int,int);
		int image_index_find_closest(double,int,int);
};

#endif

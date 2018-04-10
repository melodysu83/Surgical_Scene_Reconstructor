#ifndef MYRECONSTR_STORAGE_H
#define MYRECONSTR_STORAGE_H

#include "myproject/MyReconstr_Display.h"
class MyReconstr_Storage
{
	private:		
		int CAMERA_COUNT;

		string* IMG_FOLDER;
		string* TME_STAMP_FILE;
		string* CAM_POSE_FILE;
		string* CAM_CALI_FILE;

		vector<vector<vector<double> > > TME_STAMP_DATA;
		vector<vector<vector<double> > > CAM_POSE_DATA;
		vector<cv::Mat> CALI_INTRI_DATA;
		vector<cv::Mat> CALI_DISTO_DATA;

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
};

#endif

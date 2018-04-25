#ifndef MYRECONSTR_DISPLAY_H
#define MYRECONSTR_DISPLAY_H

#include "myproject/MyReconstr_Dependencies.h"
class MyReconstr_Display
{
	private:		
		int CAMERA_COUNT;
		bool USER_INPUT_PAUSE;
		bool USER_INPUT_SHOWSTATUS;
		bool USER_INPUT_SHOW_FEATURE_DETECTION_PARAMS;
		bool USER_INPUT_SHOW_ALL_FEATURE_DETECTION_PARAMS;
		bool USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED;
		FEATURE_ALGO_LIST MY_FEATURE_ALGO; 

		// for FAST_ALGO
		int FEATURE_PARAM_INTENSITY_THRES;
		bool FEATURE_PARAM_NON_MAX_SUPPRE;

		// for SURF_ALGO
		double FEATURE_PARAM_HESSIAN_THRES;
		int FEATURE_PARAM_N_PYRAMIDS;
		int FEATURE_PARAM_N_PYRAMID_LAYERS;
		bool FEATURE_PARAM_DESCRIPTOR_EXTENDED;

		// for ORB_ALGO
		int FEATURE_PARAM_N_FEATURES;
		float FEATURE_PARAM_SCALE;
		int FEATURE_PARAM_N_LEVELS;
		int FEATURE_PARAM_EDGE_THRES;

	public:
		MyReconstr_Display();
		~MyReconstr_Display();

		int get_key();
		void set_camera_count(int);
		void initialize_feature_detection_parameters();
		void load_feature_detection_parameters(); 
		void reset_feature_detection_parameters(); 
		void get_feature_detection_parameters(int*,bool*,double*,int*,int*,bool*,int*,float*,int*,int*);
		void set_feature_detection_algo(FEATURE_ALGO_LIST);
		FEATURE_ALGO_LIST get_feature_detection_algo();
		
		void display_system_message(int);
		void show_runtime(string, ros::Duration);
		void show_csv_data_summary(int,int,int,cv::Mat,cv::Mat);
		void show_camera_pose(double,vector<vector<double> >);
		void show_system_status(int,int);
		void show_feature_detection_parameters_fast(int,bool);
		void show_feature_detection_parameters_surf(double,int,int,bool);
		void show_feature_detection_parameters_orb(int,float,int,int);

		void no_valid_camera_info_file(int,int,int);
		void get_data_error(int);
		void visual_processing_error(int);
		void image_tool_function_error(int);
		void reset_data_pointers();
		bool check_if_paused();
		bool check_if_showstatus();
		bool check_if_display_feature_detection_parameters();
		bool check_if_display_all_feature_detection_parameters();
		bool check_if_feature_detection_parameters_updated();

		void display_menu();
		void display_feature_menu();
		void display_sub_menu_d();
		void display_sub_menu_r();
		void display_starting_message();
		void display_closing_message();
		void display_wrapping_up_message();
		SYSTEM_STATUS_LIST check_user_selection_p(int);
		SYSTEM_STATUS_LIST check_user_selection_d(int);
		SYSTEM_STATUS_LIST check_user_selection_r(int, SYSTEM_STATUS_LIST);
		SYSTEM_STATUS_LIST check_user_selection_f(int);
};

#endif

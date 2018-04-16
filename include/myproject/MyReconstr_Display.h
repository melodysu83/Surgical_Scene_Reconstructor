#ifndef MYRECONSTR_DISPLAY_H
#define MYRECONSTR_DISPLAY_H

#include "myproject/MyReconstr_Dependencies.h"
class MyReconstr_Display
{
	private:		
		int CAMERA_COUNT;
		bool USER_INPUT_PAUSE;
		bool USER_INPUT_SHOWSTATUS;
	public:
		MyReconstr_Display();
		~MyReconstr_Display();

		int get_key();
		void set_camera_count(int);
		void display_system_message(int);
		void show_csv_data_summary(int,int,int,cv::Mat,cv::Mat);
		void show_camera_pose(double,vector<vector<double> >);
		void show_system_status(int,int);
		void no_valid_camera_info_file(int,int,int);
		void get_data_error(int);
		void visual_processing_error(int);
		void reset_data_pointers();
		bool check_if_paused();
		bool check_if_showstatus();

		void display_menu();
		void display_sub_menu_d();
		void display_sub_menu_r();
		void display_starting_message();
		void display_closing_message();
		void display_wrapping_up_message();
		SYSTEM_STATUS_LIST check_user_selection_p(int);
		SYSTEM_STATUS_LIST check_user_selection_d(int);
		SYSTEM_STATUS_LIST check_user_selection_r(int, SYSTEM_STATUS_LIST);
};

#endif

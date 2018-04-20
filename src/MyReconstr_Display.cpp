#include <myproject/MyReconstr_Display.h>

MyReconstr_Display::MyReconstr_Display()
{
	this->USER_INPUT_PAUSE = false;
	this->USER_INPUT_SHOWSTATUS = false;
	this->CAMERA_COUNT = 0;
	this->MY_FEATURE_ALGO = DEFAULT_FEATURE_ALGO;
}


MyReconstr_Display::~MyReconstr_Display()
{
}


int MyReconstr_Display::get_key() 
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode 
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking 
    	//   returns EOF (-1) if no character is available 
    	character = fgetc(stdin);

   	// restore the original terminal attributes 
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}


void MyReconstr_Display::set_camera_count(int cam_cnt)
{
	this->CAMERA_COUNT = cam_cnt;
}


void MyReconstr_Display::set_feature_detection_algo(FEATURE_ALGO_LIST my_algo)
{
	this->MY_FEATURE_ALGO = my_algo;
}


FEATURE_ALGO_LIST MyReconstr_Display::get_feature_detection_algo()
{
	FEATURE_ALGO_LIST result = this->MY_FEATURE_ALGO;
	return result;
}


void MyReconstr_Display::display_system_message(int message_code)
{
	switch(message_code)
	{
		case 0:
			cout<<endl<<"[System Info] Invalid camera count. (Min of 2 required.)"<<endl<<endl;
			break;
		case 1:
			cout<<endl<<"[System Info] Loading ros parameters success!"<<endl<<endl;
			break;
		case 2:
			cout<<endl<<"[System Info] Loading data from csv files success! (Total camera count = "<<CAMERA_COUNT<<")"<<endl<<endl; 
			break;
		case 3:
			cout<<endl<<"[System Info] Waiting for ROS initialization..."<<endl<<endl; 
			break;
		case 4:
			cout<<endl<<"[System Info] System closing. Joining all active threads."<<endl<<endl;
			break;
		case 5:
			cout<<endl<<"[System Info] cv_bridge exception: ";
			break;
		case 6:
			cout<<endl<<"[System Info] pcl::PointCloud exception occurred."<<endl<<endl;
			break;
		case 7:
			cout<<endl<<"[System Info] Failed to init ros."<<endl<<endl;
			break;
		case 8:
			cout<<endl<<"[System Info] Camera index out of scope. (Range = 1~"<<CAMERA_COUNT<<")"<<endl<<endl;
			break;
		case 9:
			cout<<endl<<"[System Info] Ros parameter value extraction error."<<endl<<endl;
			break;
		case 10:
			cout<<endl<<"[System Info] Camera pose retrieval fail. Number of specified time entries and camera count don't match."<<endl<<endl;
			break;
		case 11:
			cout<<endl<<"[System Info] Invalid MY_FEATURE_ALGO settings."<<endl<<endl;
			break;
	}
}


void MyReconstr_Display::show_runtime(string ss, ros::Duration time_span)
{
	cout<<"Runtime for '"<<ss<<"' operation: "<<time_span<<" (sec)"<<endl;
}


void MyReconstr_Display::show_csv_data_summary(int cam_idx, int img_count, int poses_count,cv::Mat intri_data,cv::Mat disto_data)
{
	cout<<"camera"<<cam_idx<<":"<<endl;
	cout<<"    TME_STAMP_DATA:  "<<img_count<<" images"<<endl;
	cout<<"    CAM_POSE_DATA:   "<<poses_count<<" poses"<<endl;
	cout<<"    CALI_INTRI_DATA: "<<intri_data.reshape(intri_data.rows*intri_data.cols,1)<<endl;
	cout<<"    CALI_DISTO_DATA: "<<disto_data<<endl;
}


void MyReconstr_Display::show_camera_pose(double system_time, vector<vector<double> > current_cam_poses)
{
	if(current_cam_poses.size() != CAMERA_COUNT)
		get_data_error(10);

	cout<<"system time:"<<system_time<<endl;
	for(int i=0;i<CAMERA_COUNT;i++)
	{
		if(current_cam_poses[i].size() != CAM_POSE_SIZE)
			get_data_error(11);

		cout<<"cam_pose_"<<i<<" (x,y,z): "<<current_cam_poses[i][3]<<"    "<<current_cam_poses[i][7]<<"    "<<current_cam_poses[i][11]<<endl;
	}
	cout<<endl;
}


void MyReconstr_Display::show_system_status(int img_pub_cnt,int mdl_pub_cnt)
{
	cout<<endl<<"--------------System Status Update--------------"<<endl;
	cout<<"number of 2D images published: "<<img_pub_cnt<<endl;
	cout<<"number of 3D models published: "<<mdl_pub_cnt<<endl;
}


void MyReconstr_Display::no_valid_camera_info_file(int topic,int start,int end)
{
	if(topic == 1) // instrinsic
	{
		cout<<endl<<"[WARNING]: The intrinsic matrix cannot be extracted in camera_info files."<<endl;
		cout<<"It was found in starting from string index: "<<start<<" to "<<end<<"."<<endl;
		cout<<"Please recheck if the yaml files are in the correct format."<<endl;
	}
	else if(topic == 2) // distortion
	{
		cout<<endl<<"[WARNING]: The distortion matrix cannot be extracted in camera_info files."<<endl;
		cout<<"It was found in starting from string index: "<<start<<" to "<<end<<"."<<endl;
		cout<<"Please recheck if the yaml files are in the correct format."<<endl;
	}
	else
	{
		cout<<endl<<"[WARNING]: Some information cannot be extracted in camera_info files."<<endl;
		cout<<"Please recheck if the yaml files are in the correct format."<<endl;
	}
}


void MyReconstr_Display::get_data_error(int message_code)
{
	switch(message_code)
	{
		case 0:
			cout<<endl<<"[Data Warning] All specified times are invalid during camera pose extraction."<<endl<<endl;
			break;
		case 1:
			cout<<endl<<"[Data Warning] One specified time is invalid during camera pose extraction."<<endl<<endl;
			break;
		case 2:
			cout<<endl<<"[Data Warning] Data corruption! The camera pose does not have the right size."<<endl<<endl; 
			break;
		case 3:
			cout<<endl<<"[Data Warning] Requested image index out of bound."<<endl<<endl; 
			break;
		case 4:
			cout<<endl<<"[Data Warning] Requested image index will be out of bound. Please wrap up the reconstruction process soon."<<endl<<endl;
			break;
		case 5:
			cout<<endl<<"[Data Warning] Image retrival error. Specified time should be non negative."<<endl<<endl;
			break;
		case 6:
			cout<<endl<<"[Data Warning] Image retrival error. Specified time exceeded maximum time."<<endl<<endl;
			break;
		case 7:
			cout<<endl<<"[Data Warning] One requested image index invalid, replaced with blank image."<<endl<<endl;
			break;
		case 8:
			cout<<endl<<"[Data Warning] All requested image index invalid. There is something wrong with the system."<<endl<<endl;
			break;
		case 9:
			cout<<endl<<"[Data Warning] Retrieved images and camera count don't match."<<endl<<endl;
			break;
		case 10:
			cout<<endl<<"[Data Warning] Trying to display camera pose with wrong camera count."<<endl<<endl;
			break;
		case 11:
			cout<<endl<<"[Data Warning] Trying to display camera pose with wrong dimension."<<endl<<endl;
			break;
	}
}


void MyReconstr_Display::visual_processing_error(int message_code)
{
	switch(message_code)
	{
		case 0:
			cout<<endl<<"[Vision Error] 'draw_feature_points_for_images' function has mismatched images and key point lists size."<<endl<<endl;
			break;
		case 1:
			cout<<endl<<"[Vision Error] 'image_collage_maker' function expect input to be a vector of same-sized images."<<endl<<endl;
			break;
	}
}


void MyReconstr_Display::reset_data_pointers()
{
	this->USER_INPUT_PAUSE = false;
	this->USER_INPUT_SHOWSTATUS = false;
}


bool MyReconstr_Display::check_if_paused()
{
	bool check = this->USER_INPUT_PAUSE;
	return check;
}


bool MyReconstr_Display::check_if_showstatus()
{
	bool check = this->USER_INPUT_SHOWSTATUS;

	if(this->USER_INPUT_SHOWSTATUS)
		this->USER_INPUT_SHOWSTATUS = false;

	return check;
}


void MyReconstr_Display::display_menu()
{
	string ss = FEATURE_ALGO_TO_STRING(MY_FEATURE_ALGO);
	cout<<endl<<endl;

	cout<<"---------------------------Main Menu-------------------------"<<endl;
	cout<<"(D): Display collected data."<<endl;	
	cout<<"(Q): Perform 3D reconstruction in quiet mode."<<endl;
	cout<<"(R): Perform 3D reconstruction in debug mode."<<endl;
	cout<<"(F): Change Feature Detection Algorithms. (Current: "<<ss<<")"<<endl;
	cout<<"(E): System exit."<<endl<<endl<<endl;
}


void MyReconstr_Display::display_feature_menu()
{
	string ss;
	cout<<endl<<endl;
	cout<<"---------------------------Sub Menu: Feature Algo Selection-------------------------"<<endl;
	cout<<"(R): Random image feature detection with all algorithms."<<endl;	

	ss = (MY_FEATURE_ALGO == FAST_ALGO)? " <--Currently in Use":"";
	cout<<"(F): Choose the FAST Algorithm."<<ss<<endl;

	ss = (MY_FEATURE_ALGO == SURF_ALGO)? " <--Currently in Use":"";
	cout<<"(S): Choose the SURF Algorithm."<<ss<<endl;

	ss = (MY_FEATURE_ALGO == ORB_ALGO)? " <--Currently in Use":"";
	cout<<"(O): Choose the ORB Algorithm."<<ss<<endl;

	cout<<"(B): Quit and go back."<<endl<<endl<<endl;
}

void MyReconstr_Display::display_sub_menu_d()
{
	cout<<endl<<endl;
	cout<<"--------------------Sub Menu: Data Display-------------------"<<endl;
	cout<<"(S): Show current status."<<endl;
	cout<<"(R): Toggle system pause and resume."<<endl;
	cout<<"(B): Quit and go back."<<endl<<endl<<endl;
}


void MyReconstr_Display::display_sub_menu_r()
{
	cout<<endl<<endl;
	cout<<"-------------------Sub Menu: Reconstruction------------------"<<endl;
	cout<<"(S): Show current status."<<endl;	
	cout<<"(T): Toggle quiet mode and debug mode."<<endl;
	cout<<"(R): Toggle system pause and resume."<<endl;
	cout<<"(B): Quit and go back."<<endl<<endl<<endl;
}


void MyReconstr_Display::display_starting_message()
{
	cout<<"\n\n\nstart welcome!!"<<endl;
}


void MyReconstr_Display::display_closing_message()
{
	cout<<"end goodbye."<<endl;
}


void MyReconstr_Display::display_wrapping_up_message()
{
	cout<<"wrapping up reconstruction results..."<<endl;
}


SYSTEM_STATUS_LIST MyReconstr_Display::check_user_selection_p(int theKey)
{
	SYSTEM_STATUS_LIST sys_status = SYSTEM_PENDING_USER_SELECTION;
	if(theKey!=-1)
	{
		switch(theKey)
		{
			case 100: // 'd' or 'D' : display collected data
			case 68:
				cout<<endl<<"'d' Key Pressed: Display collected data."<<endl;
				sys_status = SYSTEM_DATA_DISPLAY_MODE;
				display_sub_menu_d();
				break;

			case 113: // 'q' or 'Q' : perform 3D reconstruction in quiet mode
			case 81:
				cout<<endl<<"'q' Key Pressed: Perform 3D reconstruction in quiet mode."<<endl;
				sys_status = SYSTEM_RECONSTR_QUIET_MODE;
				display_sub_menu_r();
				break;

			case 114: // 'r' or 'R' : perform 3D reconstruction in debug mode
			case 82:
				cout<<endl<<"'r' Key Pressed: Perform 3D reconstruction in debug mode."<<endl;
				sys_status = SYSTEM_RECONSTR_DEBUG_MODE;
				display_sub_menu_r();
				break;

			case 102: // 'f' or 'F' : change feature detection algorithm
			case 70:
				cout<<endl<<"'f' Key Pressed: Change Feature Detection Algorithms."<<endl;
				sys_status = SYSTEM_SHOW_FEATURE_MENU;
				break;

			case 101: // 'e' or 'E' : system exit
			case 69:
				cout<<endl<<"'e' Key Pressed: Exit."<<endl;
				sys_status = SYSTEM_EXIT_ALL;
				break;

			default:
				cout<<"'other' Key Pressed: Unrecognized option."<<endl;
				break;
		}
	}
	return sys_status;
}


SYSTEM_STATUS_LIST MyReconstr_Display::check_user_selection_d(int theKey)
{
	SYSTEM_STATUS_LIST sys_status = SYSTEM_DATA_DISPLAY_MODE;
	if(theKey!=-1)
	{
		switch(theKey)
		{
			case 115: // 's' or 'S' : show current status.
			case 83:
				cout<<endl<<"'s' Key Pressed: Show current status."<<endl;
				this->USER_INPUT_SHOWSTATUS = true;
				break;

			case 114: // 'r' or 'R' : toggle system pause and resume.
			case 82:
				cout<<endl<<"'r' Key Pressed: Toggle system pause and resume."<<endl;
				this->USER_INPUT_PAUSE = !(this->USER_INPUT_PAUSE);

				if(this->USER_INPUT_PAUSE)
					cout<<"                 System is now paused. Press 'r' again to resume."<<endl;
				else
					cout<<"                 System is now resumed."<<endl;
				break;

			case 98: // 'b' or 'B' : quit and go back.
			case 66:
				cout<<endl<<"'b' Key Pressed: Quit and go back."<<endl;
				sys_status = SYSTEM_DATA_ENDING;
				break;

			default:
				cout<<"'other' Key Pressed: Unrecognized option."<<endl;
				break;
		}
	}
	return sys_status;
}


SYSTEM_STATUS_LIST MyReconstr_Display::check_user_selection_r(int theKey, SYSTEM_STATUS_LIST sys_status)
{
	if(theKey!=-1)
	{
		switch(theKey)
		{
			case 115: // 's' or 'S' : show current status.
			case 83:
				cout<<endl<<"'s' Key Pressed: Show current status."<<endl;
				this->USER_INPUT_SHOWSTATUS = true;
				break;

			case 116: // 't' or 'T' : toggle quiet mode and debug mode.
			case 84:
				cout<<endl<<"'t' Key Pressed: Toggle quiet mode and debug mode."<<endl;
				if(sys_status == SYSTEM_RECONSTR_QUIET_MODE)
				{
					cout<<"                 System is now in debug mode."<<endl;
					sys_status = SYSTEM_RECONSTR_DEBUG_MODE;
				}
				else if(sys_status == SYSTEM_RECONSTR_DEBUG_MODE)
				{
					cout<<"                 System is now in quiet mode."<<endl;
					sys_status = SYSTEM_RECONSTR_QUIET_MODE;
					destroyWindow(OPENCV_IMSHOW_WINDOW_NAME);
					namedWindow(OPENCV_IMSHOW_WINDOW_NAME);
				}
				break;

			case 114: // 'r' or 'R' : toggle system pause and resume.
			case 82:
				cout<<endl<<"'r' Key Pressed: Toggle system pause and resume."<<endl;
				this->USER_INPUT_PAUSE = !(this->USER_INPUT_PAUSE);

				if(this->USER_INPUT_PAUSE)
					cout<<"                 System is now paused. Press 'r' again to resume."<<endl;
				else
					cout<<"                 System is now resumed."<<endl;
				break;

			case 98: // 'b' or 'B' : quit and go back.
			case 66:
				cout<<endl<<"'b' Key Pressed: Quit and go back."<<endl;
				sys_status = SYSTEM_DATA_ENDING;
				break;

			default:
				cout<<"'other' Key Pressed: Unrecognized option."<<endl;
				break;
		}
	}
	return sys_status;
}


SYSTEM_STATUS_LIST MyReconstr_Display::check_user_selection_f(int theKey)
{
	SYSTEM_STATUS_LIST sys_status = SYSTEM_PENDING_FEATURE_CHOICE;

	if(theKey!=-1)
	{
		switch(theKey)
		{
			case 114: // 'r' or 'R' : random image feature detection with all algorithms.
			case 82:
				cout<<endl<<"'r' Key Pressed: Random image feature detection with all algorithms."<<endl;
				sys_status = SYSTEM_SHOW_FEATURE_MENU;
				break;

			case 102: // 'f' or 'F' : choose the FAST Algorithm.
			case 70:
				cout<<endl<<"'f' Key Pressed: Choose the FAST Algorithm."<<endl;
				set_feature_detection_algo(FAST_ALGO);
				sys_status = SYSTEM_SHOW_MENU;
				break;

			case 115: // 's' or 'S' : choose the SURF Algorithm.
			case 83:
				cout<<endl<<"'s' Key Pressed: Choose the SURF Algorithm."<<endl;
				set_feature_detection_algo(SURF_ALGO);
				sys_status = SYSTEM_SHOW_MENU;
				break;

			case 111: // 'o' or 'O' : choose the ORB Algorithm.
			case 79:
				cout<<endl<<"'o' Key Pressed: Choose the ORB Algorithm."<<endl;
				set_feature_detection_algo(ORB_ALGO);
				sys_status = SYSTEM_SHOW_MENU;
				break;

			case 98: // 'b' or 'B' : quit and go back.
			case 66:
				cout<<endl<<"'b' Key Pressed: Quit and go back."<<endl;
				sys_status = SYSTEM_SHOW_MENU;
				break;

			default:
				cout<<"'other' Key Pressed: Unrecognized option."<<endl;
				break;
		}
	}
	return sys_status;
}



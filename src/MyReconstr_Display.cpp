#include <myproject/MyReconstr_Display.h>

MyReconstr_Display::MyReconstr_Display()
{
	this->USER_INPUT_PAUSE = false;
	this->USER_INPUT_SHOWSTATUS = false;
	this->USER_INPUT_SHOW_FEATURE_DETECTION_PARAMS = false;
	this->USER_INPUT_SHOW_ALL_FEATURE_DETECTION_PARAMS = false;
	this->USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED = false;

	this->CAMERA_COUNT = 0;
	this->MY_FEATURE_ALGO = DEFAULT_FEATURE_ALGO;
	initialize_feature_detection_parameters();
}


MyReconstr_Display::~MyReconstr_Display()
{
}


void  MyReconstr_Display::initialize_feature_detection_parameters()
{
	// for FAST_ALGO
	this->FEATURE_PARAM_INTENSITY_THRES = DEFAULT_FEATURE_PARAM_INTENSITY_THRES;
	this->FEATURE_PARAM_NON_MAX_SUPPRE = DEFAULT_FEATURE_PARAM_NON_MAX_SUPPRE;	

	// for SURF_ALGO
	this->FEATURE_PARAM_HESSIAN_THRES = DEFAULT_FEATURE_PARAM_HESSIAN_THRES;
	this->FEATURE_PARAM_N_PYRAMIDS = DEFAULT_FEATURE_PARAM_N_PYRAMIDS;
	this->FEATURE_PARAM_N_PYRAMID_LAYERS = DEFAULT_FEATURE_PARAM_N_PYRAMID_LAYERS;
	this->FEATURE_PARAM_DESCRIPTOR_EXTENDED = DEFAULT_FEATURE_PARAM_DESCRIPTOR_EXTENDED;

	// for ORB_ALGO
	this->FEATURE_PARAM_N_FEATURES = DEFAULT_FEATURE_PARAM_N_FEATURES;
	this->FEATURE_PARAM_SCALE = DEFAULT_FEATURE_PARAM_SCALE;
	this->FEATURE_PARAM_N_LEVELS = DEFAULT_FEATURE_PARAM_N_LEVELS;
	this->FEATURE_PARAM_EDGE_THRES = DEFAULT_FEATURE_PARAM_EDGE_THRES;
}


void MyReconstr_Display::load_feature_detection_parameters()
{
	destroyWindow(OPENCV_IMSHOW_WINDOW_NAME);
	namedWindow(OPENCV_IMSHOW_WINDOW_NAME);
	int c;
	cout<<endl<<"---------------------------Tune Feature Detection Parameters-------------------------"<<endl;
	for(int i=0; i<NUM_OF_FEATURE_PARAMETERS; i++)
	{
		switch(i+1)
		{
			case 1:
				cout<<"\n[for FAST_ALGO]"<<endl;
				cout<<"Parameter 1: FEATURE_PARAM_INTENSITY_THRES = "<<FEATURE_PARAM_INTENSITY_THRES<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_INTENSITY_THRES = min(FEATURE_PARAM_INTENSITY_THRES+1,5*DEFAULT_FEATURE_PARAM_INTENSITY_THRES);
							cout<<"\r>> FEATURE_PARAM_INTENSITY_THRES = "<<FEATURE_PARAM_INTENSITY_THRES<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_INTENSITY_THRES = max(FEATURE_PARAM_INTENSITY_THRES-1,0);
							cout<<"\r>> FEATURE_PARAM_INTENSITY_THRES = "<<FEATURE_PARAM_INTENSITY_THRES<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_INTENSITY_THRES = "<<FEATURE_PARAM_INTENSITY_THRES<<endl;
				break;
			case 2:
				cout<<"\nParameter 2: FEATURE_PARAM_NON_MAX_SUPPRE = "<<((FEATURE_PARAM_NON_MAX_SUPPRE)?"true ":"false")<<endl;
				cout<<"             (1) toggle parameter value."<<endl;
				cout<<"             (2) save and go to next parameter."<<endl;
				cout<<"             (3) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_NON_MAX_SUPPRE = !FEATURE_PARAM_NON_MAX_SUPPRE;
							cout<<"\r>> FEATURE_PARAM_NON_MAX_SUPPRE = "<<((FEATURE_PARAM_NON_MAX_SUPPRE)?"true ":"false")<<"\r";
						}
						else if(c == '2')
							break;
						else if(c == '3')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_NON_MAX_SUPPRE = "<<((FEATURE_PARAM_NON_MAX_SUPPRE)?"true ":"false")<<endl;
				break;
			case 3:
				cout<<"\n[for SURF_ALGO]"<<endl;
				cout<<"Parameter 3: FEATURE_PARAM_HESSIAN_THRES = "<<FEATURE_PARAM_HESSIAN_THRES<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_HESSIAN_THRES = min(int(FEATURE_PARAM_HESSIAN_THRES+10),10*DEFAULT_FEATURE_PARAM_HESSIAN_THRES/3);
							cout<<"\r>> FEATURE_PARAM_HESSIAN_THRES = "<<FEATURE_PARAM_HESSIAN_THRES<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_HESSIAN_THRES = max(int(FEATURE_PARAM_HESSIAN_THRES-10),0);
							cout<<"\r>> FEATURE_PARAM_HESSIAN_THRES = "<<FEATURE_PARAM_HESSIAN_THRES<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_HESSIAN_THRES = "<<FEATURE_PARAM_HESSIAN_THRES<<endl;
				break;
			case 4:
				cout<<"\nParameter 4: FEATURE_PARAM_N_PYRAMIDS = "<<FEATURE_PARAM_N_PYRAMIDS<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_N_PYRAMIDS = min(int(FEATURE_PARAM_N_PYRAMIDS+1),10*DEFAULT_FEATURE_PARAM_N_PYRAMIDS/4);
							cout<<"\r>> FEATURE_PARAM_N_PYRAMIDS = "<<FEATURE_PARAM_N_PYRAMIDS<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_N_PYRAMIDS = max(int(FEATURE_PARAM_N_PYRAMIDS-1),0);
							cout<<"\r>> FEATURE_PARAM_N_PYRAMIDS = "<<FEATURE_PARAM_N_PYRAMIDS<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_N_PYRAMIDS = "<<FEATURE_PARAM_N_PYRAMIDS<<endl;
				break;
			case 5:
				cout<<"\nParameter 5: FEATURE_PARAM_N_PYRAMID_LAYERS = "<<FEATURE_PARAM_N_PYRAMID_LAYERS<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_N_PYRAMID_LAYERS = min(int(FEATURE_PARAM_N_PYRAMID_LAYERS+1),10*DEFAULT_FEATURE_PARAM_N_PYRAMID_LAYERS/3);
							cout<<"\r>> FEATURE_PARAM_N_PYRAMID_LAYERS = "<<FEATURE_PARAM_N_PYRAMID_LAYERS<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_N_PYRAMID_LAYERS = max(int(FEATURE_PARAM_N_PYRAMID_LAYERS-1),0);
							cout<<"\r>> FEATURE_PARAM_N_PYRAMID_LAYERS = "<<FEATURE_PARAM_N_PYRAMID_LAYERS<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_N_PYRAMID_LAYERS = "<<FEATURE_PARAM_N_PYRAMID_LAYERS<<endl;
				break;
			case 6:
				cout<<"\nParameter 6: FEATURE_PARAM_DESCRIPTOR_EXTENDED = "<<((FEATURE_PARAM_DESCRIPTOR_EXTENDED)?"true ":"false")<<endl;
				cout<<"             (1) toggle parameter value."<<endl;
				cout<<"             (2) save and go to next parameter."<<endl;
				cout<<"             (3) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_DESCRIPTOR_EXTENDED = !FEATURE_PARAM_DESCRIPTOR_EXTENDED;
							cout<<"\r>> FEATURE_PARAM_DESCRIPTOR_EXTENDED = "<<((FEATURE_PARAM_DESCRIPTOR_EXTENDED)?"true ":"false")<<"\r";
						}
						else if(c == '2')
							break;
						else if(c == '3')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_DESCRIPTOR_EXTENDED = "<<((FEATURE_PARAM_DESCRIPTOR_EXTENDED)?"true ":"false")<<endl;
				break;
				break;
			case 7:	
				cout<<"\n[for ORB_ALGO]"<<endl;
				cout<<"Parameter 7: FEATURE_PARAM_N_FEATURES = "<<FEATURE_PARAM_N_FEATURES<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_N_FEATURES = min(int(FEATURE_PARAM_N_FEATURES+100),5*DEFAULT_FEATURE_PARAM_N_FEATURES);
							cout<<"\r>> FEATURE_PARAM_N_FEATURES = "<<FEATURE_PARAM_N_FEATURES<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_N_FEATURES = max(int(FEATURE_PARAM_N_FEATURES-100),0);
							cout<<"\r>> FEATURE_PARAM_N_LEVELS = "<<FEATURE_PARAM_N_FEATURES<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_N_FEATURES = "<<FEATURE_PARAM_N_FEATURES<<endl;
				break;
			case 8:
				cout<<"\nParameter 8: FEATURE_PARAM_SCALE = "<<FEATURE_PARAM_SCALE<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_SCALE = FEATURE_PARAM_SCALE+0.1;
							if(FEATURE_PARAM_SCALE > 3.0)
								FEATURE_PARAM_SCALE = 3.0;
							cout<<"\r>> FEATURE_PARAM_SCALE = "<<FEATURE_PARAM_SCALE<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_SCALE = FEATURE_PARAM_SCALE-0.1;
							if(FEATURE_PARAM_SCALE < 0.0)
								FEATURE_PARAM_SCALE = 0.0;
							cout<<"\r>> FEATURE_PARAM_SCALE = "<<FEATURE_PARAM_SCALE<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_SCALE = "<<FEATURE_PARAM_SCALE<<endl;
				break;
			case 9:
				cout<<"\nParameter 9: FEATURE_PARAM_N_LEVELS = "<<FEATURE_PARAM_N_LEVELS<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and go to next parameter."<<endl;
				cout<<"             (4) exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_N_LEVELS = min(int(FEATURE_PARAM_N_LEVELS+1),2*DEFAULT_FEATURE_PARAM_N_LEVELS);
							cout<<"\r>> FEATURE_PARAM_N_LEVELS = "<<FEATURE_PARAM_N_LEVELS<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_N_LEVELS = max(int(FEATURE_PARAM_N_LEVELS-1),1);
							cout<<"\r>> FEATURE_PARAM_N_LEVELS = "<<FEATURE_PARAM_N_LEVELS<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_N_LEVELS = "<<FEATURE_PARAM_N_LEVELS<<endl;
				break;
			case 10:
				cout<<"\nParameter 10: FEATURE_PARAM_EDGE_THRES = "<<FEATURE_PARAM_EDGE_THRES<<" (current value)"<<endl;
				cout<<"             (1) to increase value."<<endl;
				cout<<"             (2) to decrease value."<<endl;
				cout<<"             (3) save and exit."<<endl;
				cout<<"Please type in your choice:"<<endl;
				
				while(true)
				{		
					c = get_key();
					if(c != -1)
					{			
						if(c == '1')
						{
							FEATURE_PARAM_EDGE_THRES = min(int(FEATURE_PARAM_EDGE_THRES+1),20*DEFAULT_FEATURE_PARAM_EDGE_THRES/11);
							cout<<"\r>> FEATURE_PARAM_EDGE_THRES = "<<FEATURE_PARAM_EDGE_THRES<<"\r";
						}
						else if(c == '2')
						{
							FEATURE_PARAM_EDGE_THRES = max(int(FEATURE_PARAM_EDGE_THRES-1),1);
							cout<<"\r>> FEATURE_PARAM_EDGE_THRES = "<<FEATURE_PARAM_EDGE_THRES<<"\r";
						}
						else if(c == '3')
							break;
						else if(c == '4')
						{
							i = NUM_OF_FEATURE_PARAMETERS;
							break;
						}
					}
					
				}
				cout<<">> FEATURE_PARAM_EDGE_THRES = "<<FEATURE_PARAM_EDGE_THRES<<endl;
				cout<<"\n All done!~"<<endl;
				break;
		}
	}

}


void MyReconstr_Display::reset_feature_detection_parameters()
{
	// for FAST_ALGO
	this->FEATURE_PARAM_INTENSITY_THRES = DEFAULT_FEATURE_PARAM_INTENSITY_THRES;
	this->FEATURE_PARAM_NON_MAX_SUPPRE = DEFAULT_FEATURE_PARAM_NON_MAX_SUPPRE;	

	// for SURF_ALGO
	this->FEATURE_PARAM_HESSIAN_THRES = DEFAULT_FEATURE_PARAM_HESSIAN_THRES;
	this->FEATURE_PARAM_N_PYRAMIDS = DEFAULT_FEATURE_PARAM_N_PYRAMIDS;
	this->FEATURE_PARAM_N_PYRAMID_LAYERS = DEFAULT_FEATURE_PARAM_N_PYRAMID_LAYERS;
	this->FEATURE_PARAM_DESCRIPTOR_EXTENDED = DEFAULT_FEATURE_PARAM_DESCRIPTOR_EXTENDED;

	// for ORB_ALGO
	this->FEATURE_PARAM_N_FEATURES = DEFAULT_FEATURE_PARAM_N_FEATURES;
	this->FEATURE_PARAM_SCALE = DEFAULT_FEATURE_PARAM_SCALE;
	this->FEATURE_PARAM_N_LEVELS = DEFAULT_FEATURE_PARAM_N_LEVELS;
	this->FEATURE_PARAM_EDGE_THRES = DEFAULT_FEATURE_PARAM_EDGE_THRES;
}


void MyReconstr_Display::get_feature_detection_parameters(int* p1,bool* p2,double* p3,int* p4,int* p5,bool* p6,int* p7,float* p8,int* p9,int* p10)
{
	*p1 = FEATURE_PARAM_INTENSITY_THRES;
	*p2 = FEATURE_PARAM_NON_MAX_SUPPRE;
	*p3 = FEATURE_PARAM_HESSIAN_THRES;
	*p4 = FEATURE_PARAM_N_PYRAMIDS;
	*p5 = FEATURE_PARAM_N_PYRAMID_LAYERS;
	*p6 = FEATURE_PARAM_DESCRIPTOR_EXTENDED;
	*p7 = FEATURE_PARAM_N_FEATURES;
	*p8 = FEATURE_PARAM_SCALE;
	*p9 = FEATURE_PARAM_N_LEVELS;
	*p10 = FEATURE_PARAM_EDGE_THRES;
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


void MyReconstr_Display::show_feature_detection_parameters_fast(int param1,bool param2)
{
	cout<<endl<<"[FEATURE DETECTION PARAMETERS] -"<<FEATURE_ALGO_TO_STRING((FEATURE_ALGO_LIST)0)<<endl;
	cout<<"FEATURE_PARAM_INTENSITY_THRES: "<<param1<<endl;
	cout<<"FEATURE_PARAM_NON_MAX_SUPPRE:  "<<param2<<endl;
}


void MyReconstr_Display::show_feature_detection_parameters_surf(double param1,int param2,int param3,bool param4)
{
	cout<<endl<<"[FEATURE DETECTION PARAMETERS] -"<<FEATURE_ALGO_TO_STRING((FEATURE_ALGO_LIST)1)<<endl;
	cout<<"FEATURE_PARAM_HESSIAN_THRES:       "<<param1<<endl;
	cout<<"FEATURE_PARAM_N_PYRAMIDS:          "<<param2<<endl;
	cout<<"FEATURE_PARAM_N_PYRAMID_LAYERS:    "<<param3<<endl;
	cout<<"FEATURE_PARAM_DESCRIPTOR_EXTENDED: "<<param4<<endl;
}


void MyReconstr_Display::show_feature_detection_parameters_orb(int param1,float param2,int param3,int param4)
{
	cout<<endl<<"[FEATURE DETECTION PARAMETERS] -"<<FEATURE_ALGO_TO_STRING((FEATURE_ALGO_LIST)2)<<endl;
	cout<<"FEATURE_PARAM_N_FEATURES: "<<param1<<endl;
	cout<<"FEATURE_PARAM_SCALE:      "<<param2<<endl;
	cout<<"FEATURE_PARAM_N_LEVELS:   "<<param3<<endl;
	cout<<"FEATURE_PARAM_EDGE_THRES: "<<param4<<endl;
}


void MyReconstr_Display::show_all_camera_combinations(vector<vector<int> > comb)
{
	cout<<endl<<"[CAMERA_MATCHES_COMBINATION] "<<endl;
	for(int i=0; i<comb.size(); i++)
	{
		cout<<"comb "<<i<<": [";
		for(int j=0; j<comb[i].size()-1;j++)
			cout<<"cam_"<<comb[i][j]<<",";
		cout<<"cam_"<<comb[i][comb[i].size()-1]<<"]"<<endl;
	}
	cout<<endl;
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
		case 2:
			cout<<endl<<"[Vision Error] 'process_image2D' dimension of last two arguements doesn't match the camera count."<<endl<<endl;
			break;
		case 3:
			cout<<endl<<"[Vision Error] 'process_image2D' dimension error for camera pose (should have 16 columns)."<<endl<<endl;
			break;
		case 4:
			cout<<endl<<"[Vision Error] 'feature_tracking_intra_camera' encountered memory stack error (empty stack)."<<endl<<endl;
			break;
		case 5:
			cout<<endl<<"[Vision Error] 'feature_tracking_intra_camera' camera count mismatch."<<endl<<endl;
			break;
		case 6:
			cout<<endl<<"[Vision Error] 'feature_tracking_cross_camera' output match points dimension mismatched."<<endl<<endl;
			break;
		case 7:
			cout<<endl<<"[Vision Error] 'draw_feature_intra_cam_matches_for_images' number of images doesn't match number of feature point matches sets."<<endl<<endl;
			break;
		case 8:
			cout<<endl<<"[Vision Error] 'draw_feature_inter_cam_matches_for_images' number of cam matches comb doesn't match number of inter cam feature matches sets."<<endl<<endl;
			break;
		case 9:
			cout<<endl<<"[Vision Error] 'draw_feature_inter_cam_matches_for_images' number of images doesn't match number of feature point matches sets."<<endl<<endl;
			break;
		case 10:
			cout<<endl<<"[Vision Error] 'draw_feature_intra_cam_matches' number of old and new feature points don't match."<<endl<<endl;
			break;
	}
}


void MyReconstr_Display::image_tool_function_error(int message_code)
{
	switch(message_code)
	{
		case 0:
			cout<<endl<<"[Image Tool Error] 'norm_1' requires a column vector of data type cv::Mat."<<endl<<endl;
			break;
		case 1:
			cout<<endl<<"[Image Tool Error] 'norm_2' requires a column vector of data type cv::Mat."<<endl<<endl;
			break;
		case 2:
			cout<<endl<<"[Image Tool Error] 'project_Wpt_to_Ipt' function has invalid arguement dimension."<<endl<<endl;
			break;
		case 3:
			cout<<endl<<"[Image Tool Error] 'compute_projection_jacobian_J' expects arguement of size (2x3) or (3x4)."<<endl<<endl;
			break;
		case 4:
			cout<<endl<<"[Image Tool Error] 'evaluate_Ipt_to_Mpt_correspondance' one or more arguements of invalid shape."<<endl<<endl;
			break;
		case 5:
			cout<<endl<<"[Image Tool Error] 'to_scalable_form' function has incorrect return shape."<<endl<<endl;
			break;
		case 6:
			cout<<endl<<"[Image Tool Error] 'to_normalized_form' trying to normalize by 0 factor."<<endl<<endl;
			break;
		case 7:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_Ipt' has incorrect input shape."<<endl<<endl;
			break;
		case 8:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_Mpt' has incorrect input shape."<<endl<<endl;
			break;
		case 9:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_P' expects input size (2x3) or (3x4)."<<endl<<endl;
			break;
		case 10:
			cout<<endl<<"[Image Tool Error] 'check_if_normalized_Ipt' has incorrect input shape."<<endl<<endl;
			break;
		case 11:
			cout<<endl<<"[Image Tool Error] 'check_if_normalized_Mpt' has incorrect input shape."<<endl<<endl;
			break;
		case 12:
			cout<<endl<<"[Image Tool Error] 'check_if_normalized_P' expects input size (2x3) or (3x4)."<<endl<<endl;
			break;
		case 13:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_J' expects input size (2x3)."<<endl<<endl;
			break;
		case 14:
			cout<<endl<<"[Image Tool Error] 'compute_Mpt_uncertainty_covariance_E' has invalid J input shape."<<endl<<endl;
			break;
		case 15:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_E' has invalid E input shape."<<endl<<endl;
			break;
		case 16:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_c' has invalid c input shape."<<endl<<endl;
			break;
		case 17:
			cout<<endl<<"[Image Tool Error] 'mahalanobis_distance(pti,ptj,Ei,Ji)' incorrect input argument shape."<<endl<<endl;
			break;
		case 18:
			cout<<endl<<"[Image Tool Error] 'mahalanobis_distance(pti,ptj,ci)' incorrect input argument shape."<<endl<<endl;
			break;
		case 19:
			cout<<endl<<"[Image Tool Error] 'update_Mpt_from_Ipt' something wrong with thw Kalman filter gain K."<<endl<<endl;
			break;
		case 20:
			cout<<endl<<"[Image Tool Error] 'update_Mpt_from_Ipt' invalid projection matrix P."<<endl<<endl;
			break;
		case 21:
			cout<<endl<<"[Image Tool Error] 'update_Mpt_from_Ipt' either 3D  or 2D point has incorrect shape."<<endl<<endl;
			break;
		case 22:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_K' has invalid K shape."<<endl<<endl;
			break;
		case 23:
			cout<<endl<<"[Image Tool Error] 'update_Mpt_from_Ipt' output size incorrect, something is wrong."<<endl<<endl;
			break;
		case 24:
			cout<<endl<<"[Image Tool Error] 'update_kalman_gain' input size incorrect, something is wrong."<<endl<<endl;
			break;
		case 25:
			cout<<endl<<"[Image Tool Error] 'update_kalman_gain' output size incorrect, something is wrong."<<endl<<endl;
			break;
		case 26:
			cout<<endl<<"[Image Tool Error] 'update_Mpt_uncertainty_covariance_E' input size incorrect, something is wrong."<<endl<<endl;
			break;
		case 27:
			cout<<endl<<"[Image Tool Error] 'update_Mpt_uncertainty_covariance_E' output size incorrect, something is wrong."<<endl<<endl;
			break;
		case 28:
			cout<<endl<<"[Image Tool Error] 'check_if_valid_Js' expects input size ((2k)x3)."<<endl<<endl;
			break;
		case 29:
			cout<<endl<<"[Image Tool Error] 'factorial' expects positive integer input."<<endl<<endl;
			break;
		case 30:
			cout<<endl<<"[Image Tool Error] 'constrained_combination' number of output pairs is not correct."<<endl<<endl;
			break;
		case 31:
			cout<<endl<<"[Image Tool Error] 'poses_to_transMats' elements of input vector are expected to be of size (1x6) or size (1x16)."<<endl<<endl;
			break;
		case 32:
			cout<<endl<<"[Image Tool Error] 'poses_to_transMats' output is incorrect."<<endl<<endl;
			break;
		case 33:
			cout<<endl<<"[Image Tool Error] 'transMats_to_poses' elements of input vector are expected to be of size (4x4)."<<endl<<endl;
			break;
		case 34:
			cout<<endl<<"[Image Tool Error] 'transMats_to_poses' output is incorrect.."<<endl<<endl;
			break;
		case 35:
			cout<<endl<<"[Image Tool Error] 'pose_to_transMat' expect input to have 6 entries."<<endl<<endl;
			break;
		case 36:
			cout<<endl<<"[Image Tool Error] 'transMat_to_pose' expect input to be (4x4)."<<endl<<endl;
			break;
		case 37:
			cout<<endl<<"[Image Tool Error] 'poses_to_projMats' elements of input vector are expected to be of size (1x6) or size (1x16)."<<endl<<endl;
			break;
		case 38:
			cout<<endl<<"[Image Tool Error] 'poses_to_projMats' output is incorrect."<<endl<<endl;
			break;
		case 39:
			cout<<endl<<"[Image Tool Error] 'projMats_to_poses' elements of input vector are expected to be of size (3x4)."<<endl<<endl;
			break;
		case 40:
			cout<<endl<<"[Image Tool Error] 'projMats_to_poses' output is incorrect.."<<endl<<endl;
			break;
		case 41:
			cout<<endl<<"[Image Tool Error] 'pose_to_projMat' expect input to have 6 entries."<<endl<<endl;
			break;
		case 42:
			cout<<endl<<"[Image Tool Error] 'projMat_to_pose' expect input to be (3x4)."<<endl<<endl;
			break;
		case 43:
			cout<<endl<<"[Image Tool Error] 'pose_to_transMat' expect input to have 16 entries."<<endl<<endl;
			break;
		case 44:
			cout<<endl<<"[Image Tool Error] 'pose_to_projMat' expect input to have 16 entries."<<endl<<endl;
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


bool MyReconstr_Display::check_if_display_feature_detection_parameters()
{
	bool check = this->USER_INPUT_SHOW_FEATURE_DETECTION_PARAMS;

	if(this->USER_INPUT_SHOW_FEATURE_DETECTION_PARAMS)
		this->USER_INPUT_SHOW_FEATURE_DETECTION_PARAMS = false;

	return check;
}


bool MyReconstr_Display::check_if_display_all_feature_detection_parameters()
{
	bool check = this->USER_INPUT_SHOW_ALL_FEATURE_DETECTION_PARAMS;

	if(this->USER_INPUT_SHOW_ALL_FEATURE_DETECTION_PARAMS)
		this->USER_INPUT_SHOW_ALL_FEATURE_DETECTION_PARAMS = false;

	return check;
}


bool MyReconstr_Display::check_if_feature_detection_parameters_updated()
{
	bool check = this->USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED;

	if(this->USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED)
		this->USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED = false;

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

	cout<<"(D): Display current feature detection parameters."<<endl;
	cout<<"(A): Display all feature detection parameters."<<endl;
	cout<<"(C): Cancel all feature detection parameter adjustments. (reset)"<<endl;
	cout<<"(T): Tune current feature detection parameters."<<endl;
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

			case 100: // 'd' or 'D' : display current feature detection parameters.
			case 68:
				cout<<endl<<"'d' Key Pressed: Display current feature detection parameters."<<endl;
				this->USER_INPUT_SHOW_FEATURE_DETECTION_PARAMS = true;
				sys_status = SYSTEM_SHOW_FEATURE_MENU;
				break;

			case 97: // 'a' or 'A' : display all feature detection parameters.
			case 65:
				cout<<endl<<"'a' Key Pressed: Display all feature detection parameters."<<endl;
				this->USER_INPUT_SHOW_ALL_FEATURE_DETECTION_PARAMS = true;
				sys_status = SYSTEM_SHOW_FEATURE_MENU;
				break;

			case 99: // 'c' or 'C' : cancel all feature detection parameter adjustments.
			case 67:
				cout<<endl<<"'c' Key Pressed: Cancel all feature detection parameter adjustments."<<endl;
				reset_feature_detection_parameters();
				this->USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED = true;
				sys_status = SYSTEM_SHOW_FEATURE_MENU;
				break;

			case 116: // 't' or 'T' : tune current feature detection parameters.
			case 84:
				cout<<endl<<"'t' Key Pressed: Tune current feature detection parameters."<<endl;
				load_feature_detection_parameters();
				this->USER_INPUT_FEATURE_DETECTION_PARAMS_UPDATED = true;
				sys_status = SYSTEM_SHOW_FEATURE_MENU;
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



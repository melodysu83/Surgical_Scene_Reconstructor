#include <myproject/MyReconstr_Display.h>

MyReconstr_Display::MyReconstr_Display()
{
	// any init steps
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
			cout<<endl<<"[System Info] plc::PointCloud exception occurred."<<endl<<endl;
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
	}
}


void MyReconstr_Display::show_csv_data_summary(int cam_idx, int img_count, int poses_count,cv::Mat intri_data,cv::Mat disto_data)
{
	cout<<"camera"<<cam_idx<<":"<<endl;
	cout<<"    TME_STAMP_DATA:  "<<img_count<<" images"<<endl;
	cout<<"    CAM_POSE_DATA:   "<<poses_count<<" poses"<<endl;
	cout<<"    CALI_INTRI_DATA: "<<intri_data.reshape(intri_data.rows*intri_data.cols,1)<<endl;
	cout<<"    CALI_DISTO_DATA: "<<disto_data<<endl;
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
	}
}

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
			cout<<endl<<"[System Info] Loading data from csv files success!"<<endl<<endl; 
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
	}
}


void MyReconstr_Display::show_csv_data_summary(int cam_idx, int img_count, int poses_count)
{
	cout<<"camera"<<cam_idx<<":"<<endl;
	cout<<"    TME_STAMP_DATA: "<<img_count<<" images"<<endl;
	cout<<"    CAM_POSE_DATA:  "<<poses_count<<" poses"<<endl;
}




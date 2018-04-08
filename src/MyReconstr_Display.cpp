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
			system("[System Info] Invalid camera count. (Min of 2 required.)");
			break;
		case 1:
			system("[System Info] Loading ros parameters success!");
			break;
		case 2:
			system("[System Info] Loading data from csv files success!"); 
			break;
		case 3:
			system("[System Info] Waiting for ROS initialization..."); 
			break;
		case 4:
			system("[System Info] System closing. Joining all active threads.");
			break;
		case 5:
			cout<<"cv_bridge exception: ";
			break;
		case 6:
			system("[System Info] plc::PointCloud exception occurred.");
			break;
		case 7:
			system("[System Info] Failed to init ros.");
			break;
	}
}

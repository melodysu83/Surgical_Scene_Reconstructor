#ifndef MYRECONSTR_DISPLAY_H
#define MYRECONSTR_DISPLAY_H

#include "myproject/MyReconstr_Dependencies.h"
class MyReconstr_Display
{
	private:		
		int CAMERA_COUNT;
	public:
		MyReconstr_Display();
		~MyReconstr_Display();

		int get_key();
		void set_camera_count(int);
		void display_system_message(int);
};

#endif

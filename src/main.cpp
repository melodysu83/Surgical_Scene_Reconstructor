#include "myproject/MyReconstr_Dependencies.h"

int main(int argc, char **argv)
{
        // Initialize ROS node
        ros::init(argc, argv, "Surgical_Scene_Reconstructor");

        // initialize the system
        MyReconstr_Controller ctrl(argc,argv);

        // start the console_thread and ros_thread
        ctrl.start_thread();

        // trigger ROS publish and subscribe update
        ros::spin();

        // join the console_thread and ros_thread
        ctrl.join_thread();
	/*
	cout<<"Hello world hi there! boooo"<<endl;
	Mat img_raw;
	img_raw = imread("/media/melody/D\ folder/surgical\ scene_reconstruction/3.\ image_files_processed/surgical_scene_melody/left/05986.png");
        namedWindow( "booo", WINDOW_AUTOSIZE ); // Create a window for display.
	while(1)
	{
		imshow("booo",img_raw);
		cv::waitKey(0);
                usleep(3000000); //3 secs
	}
	*/
        exit(1);
}


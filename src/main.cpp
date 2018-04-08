#include "myproject/MyReconstr_Controller.h"

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
        exit(1);
}


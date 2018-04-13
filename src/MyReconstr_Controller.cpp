#include <myproject/MyReconstr_Controller.h>

MyReconstr_Controller::MyReconstr_Controller(int argc, char** argv):it_(nh_)
{
	load_ros_param();  // load ROS parameters from launch file	
	load_dataset();	   // load the dataset from the csv and yaml files

	init_sys();		 // init the controller object
	if(!init_ros(argc,argv)) // initialize ros
	{	
		CONSOLE.display_system_message(7);
		exit(1);
	}
}


MyReconstr_Controller::~MyReconstr_Controller()
{
}


void MyReconstr_Controller::load_ros_param()
{                                             
	nh_.param(ROS_PARAMETER_NAME0, CAMERA_COUNT, 0);

	if(CAMERA_COUNT<2)
	{
		CONSOLE.display_system_message(0);
		exit(1);
	}
	else
	{
		CONSOLE.set_camera_count(CAMERA_COUNT);
		DATABANK.set_camera_count(CAMERA_COUNT);

		for(int i=1; i<=CAMERA_COUNT; i++)
		{
			vector<string> param_val(4);
			vector<char*> param_name = DATABANK.get_ros_param_name(i);

			for(int j=0; j<4; j++)
				nh_.param<string>(param_name[j], param_val[j],"None");
			
			DATABANK.set_ros_param_val(i,param_val);
		}
		
		CONSOLE.display_system_message(1);
	}
}


void MyReconstr_Controller::load_dataset()
{
	DATABANK.load_dataset(); 
	DATABANK.verify_loaded_dataset();
}


void MyReconstr_Controller::init_sys()
{
	this->IMAGE_PUB_COUNT = 0;
	this->MODEL_PUB_COUNT = 0;
	this->SYSTEM_TIME = 0.0;

	this->NEW_IMAGE_TO_PUB = false;
	this->NEW_MODEL_TO_PUB = false;
	this->IMAGE_STATE = IMAGE_EMPTY;
	this->MODEL_STATE = MODEL_EMPTY;
	this->SYSTEM_STATE = SYSTEM_JUST_STARTING;

	this->GOODBYE = false;
	this->USER_INPUT_PAUSE = false;
	this->USER_INPUT_SHOWSTATUS = false;
	
	namedWindow( OPENCV_IMSHOW_WINDOW_NAME, WINDOW_AUTOSIZE ); // Create a window for display.
}


bool MyReconstr_Controller::init_ros(int argc, char** argv)
{
	while(!ros::ok())
		CONSOLE.display_system_message(3);

	// Setup Image publish/subscribe relation
	image_pub_ = it_.advertise(ROS_TOPIC_PUBLISH_NAME1, 1); 
	model_pub_ = nh_.advertise<PointCloud<PointXYZRGB> >(ROS_TOPIC_PUBLISH_NAME2, 100);
	
	//ToDo: if allow realtime, need to subscribe to image ros topics
	return true;	
}


void MyReconstr_Controller::start_thread()
{
	pthread_create(&console_thread,NULL,MyReconstr_Controller::static_console_process,this);
	pthread_create(&io_thread,NULL,MyReconstr_Controller::static_io_process,this);
	pthread_create(&reconstr_thread,NULL,MyReconstr_Controller::static_reconstr_process,this);
}


void MyReconstr_Controller::join_thread()
{
	pthread_join(console_thread,NULL);
	pthread_join(io_thread,NULL);
	pthread_join(reconstr_thread,NULL);

	signal(SIGINT,sigint_handler);
	CONSOLE.display_system_message(4);
}


void MyReconstr_Controller::sigint_handler(int param)
{
  	exit(1);
}


void *MyReconstr_Controller::console_process()
{
	/*
	This is the SYSTEM_STATUS_LIST:
	--------------------------------------------
	SYSTEM_JUST_STARTING,                   // 0
	SYSTEM_SHOW_MENU,                       // 1
	SYSTEM_PENDING_USER_SELECTION,          // 2
	SYSTEM_DATA_DISPLAY_MODE,	        // 3
	SYSTEM_RECONSTR_QUIET_MODE, 		// 4
	SYSTEM_RECONSTR_DEBUG_MODE,		// 5
	SYSTEM_DATA_ENDING,			// 6
	SYSTEM_EXIT_ALL,                        // 7
	*/
	int theKey;
	static ros::Rate loop_rate(CONSOLE_LOOP_RATE);
	while (ros::ok() && !GOODBYE)
  	{
		switch(SYSTEM_STATE)
		{
			case SYSTEM_JUST_STARTING: // 0
				CONSOLE.display_starting_message();
				SYSTEM_STATE = SYSTEM_SHOW_MENU;
				break;

			case SYSTEM_SHOW_MENU: // 1
				CONSOLE.display_menu();
				clean_up_and_ready_for_restart();
				SYSTEM_STATE = SYSTEM_PENDING_USER_SELECTION;
				break;
			
			case SYSTEM_PENDING_USER_SELECTION: // 2
				theKey = CONSOLE.get_key();
				SYSTEM_STATE = CONSOLE.check_user_selection_p(theKey);
				break;

			case SYSTEM_DATA_DISPLAY_MODE: // 3
				theKey = CONSOLE.get_key();
				SYSTEM_STATE = CONSOLE.check_user_selection_d(theKey);
				USER_INPUT_PAUSE = CONSOLE.check_if_paused();
				USER_INPUT_SHOWSTATUS = CONSOLE.check_if_showstatus();
				break;	

			case SYSTEM_RECONSTR_QUIET_MODE: // 4	
				theKey = CONSOLE.get_key();
				SYSTEM_STATE = CONSOLE.check_user_selection_r(theKey,SYSTEM_STATE);
				USER_INPUT_PAUSE = CONSOLE.check_if_paused();
				USER_INPUT_SHOWSTATUS = CONSOLE.check_if_showstatus();
				break;	

			case SYSTEM_RECONSTR_DEBUG_MODE: // 5	
				theKey = CONSOLE.get_key();
				SYSTEM_STATE = CONSOLE.check_user_selection_r(theKey,SYSTEM_STATE);
				USER_INPUT_PAUSE = CONSOLE.check_if_paused();
				USER_INPUT_SHOWSTATUS = CONSOLE.check_if_showstatus();
				break;	

			case SYSTEM_DATA_ENDING: // 6	
				
				CONSOLE.display_wrapping_up_message();
				final_result_publish_and_display();
				SYSTEM_STATE = SYSTEM_SHOW_MENU;
				break;	

			case SYSTEM_EXIT_ALL: // 7	
				GOODBYE = true;
				CONSOLE.display_closing_message();
				exit(1);
				break;	
		}

		if(USER_INPUT_SHOWSTATUS)
		{
			//ToDo: Show status
			//CONSOLE.show_system_status();
			CONSOLE.show_camera_pose(SYSTEM_TIME,CURRENT_CAM_POSES);
		}

		loop_rate.sleep();
		ros::spinOnce();  
	}
}


void *MyReconstr_Controller::io_process()
{
	static ros::Rate loop_rate(IO_LOOP_RATE);

	while (ros::ok() && !GOODBYE)
  	{
		bool ready_to_load_data =  ((SYSTEM_STATE == SYSTEM_DATA_DISPLAY_MODE) && (IMAGE_STATE == IMAGE_DONE_PUBLISHING || IMAGE_STATE == IMAGE_EMPTY))
				        || ((SYSTEM_STATE == SYSTEM_RECONSTR_QUIET_MODE) && (IMAGE_STATE == IMAGE_DONE_PROCESSING || IMAGE_STATE == IMAGE_EMPTY))
					|| ((SYSTEM_STATE == SYSTEM_RECONSTR_DEBUG_MODE) && (IMAGE_STATE == IMAGE_DONE_PUBLISHING || IMAGE_STATE == IMAGE_EMPTY));

		bool ready_to_publish_data =  ((SYSTEM_STATE == SYSTEM_DATA_DISPLAY_MODE) || (SYSTEM_STATE == SYSTEM_RECONSTR_DEBUG_MODE))
					   && ((IMAGE_STATE == IMAGE_DONE_PROCESSING) || (MODEL_STATE == MODEL_DONE_PROCESSING));

		// part 1: load data
		if(ready_to_load_data && !USER_INPUT_PAUSE)
		{
			this->IMAGE_STATE = IMAGE_START_LOADING;

			// (1) load current images/pose
			load_currect_images_and_pose();

			// (2) update system time
			this->SYSTEM_TIME = this->SYSTEM_TIME+0.01; //ToDo: Find a way to determine termination

			this->IMAGE_STATE = IMAGE_DONE_LOADING;
		}

		// part 2: publish data
		if(ready_to_publish_data && !USER_INPUT_PAUSE)
		{
			imagePb(); // publish to image2D
			modelPb(); // publish to model3D
			
			//ToDo: display by image show too?
			imshow(OPENCV_IMSHOW_WINDOW_NAME,Images2D);
			cv::waitKey(30);
		}

		ros::spinOnce();   // ensures that the callback gets called
		loop_rate.sleep();
	}
}


void *MyReconstr_Controller::reconstr_process()
{
	while (ros::ok() && !GOODBYE)
  	{
		bool ready_to_reconstr_data = ((SYSTEM_STATE == SYSTEM_RECONSTR_QUIET_MODE) || (SYSTEM_STATE == SYSTEM_RECONSTR_DEBUG_MODE))
                                           && (IMAGE_STATE == IMAGE_DONE_LOADING);

		bool ready_to_process_data =  ready_to_reconstr_data || ((SYSTEM_STATE == SYSTEM_DATA_DISPLAY_MODE) && (IMAGE_STATE == IMAGE_DONE_LOADING));


		if(ready_to_process_data && !USER_INPUT_PAUSE)
		{
			Model3D_Last.points.resize(Model3D.points.size());
			copyPointCloud(Model3D, Model3D_Last);

			Images2D_Last = Images2D.clone();

			this->IMAGE_STATE = IMAGE_START_PROCESSING;
			this->MODEL_STATE = MODEL_START_PROCESSING;

			// (1) feature extraction: compute for Images2D
			// ToDo: Replace the following baby script with real stuff!
			int r=10;
			int c=24;
			int b=1; // B,G,R

			Images2D = CURRENT_IMAGES[0].clone();
			Images2D.at<cv::Vec3b>(r,c)[b] = 27;

			this->NEW_IMAGE_TO_PUB = true;

			if(ready_to_reconstr_data && !USER_INPUT_PAUSE) // ToDo: do 3D reconstruction!
			{
				// (2) feature matching
			
				// (3) camera grouping and map building...?

				// (4) compute for Model3D
				// ToDo: Replace the following baby script with real stuff!
				// ref: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html

				int num_of_pts = 2;
				Model3D.clear(); // Removes all points in a cloud and sets the width and height to 0. 
				Model3D.points.resize (num_of_pts);
				Model3D.width = num_of_pts;
				Model3D.height = 1;
				PointXYZRGB pt1, pt2;
				pt1.x = 1;	pt1.y = 2;	pt1.z = 5;	pt1.r = 100;	pt1.g = 200;	pt1.b = 125;
				pt2.x = 1;	pt2.y = 0.7;	pt2.z = 5;  	pt2.r = 123;	pt2.g = 12;	pt2.b = 50;
				Model3D.points.push_back(pt1);
				Model3D.points.push_back(pt2);

				// (5) set flags
				this->NEW_MODEL_TO_PUB = true;
				this->MODEL_STATE = MODEL_DONE_PROCESSING;
			}

			// (6) set more flags
			this->IMAGE_STATE = IMAGE_DONE_PROCESSING;
			
		}
	}
}


void * MyReconstr_Controller::static_console_process(void* classRef)
{
	return ((MyReconstr_Controller *)classRef)->console_process();
}


void * MyReconstr_Controller::static_io_process(void* classRef)
{
	return ((MyReconstr_Controller *)classRef)->io_process();
}


void * MyReconstr_Controller::static_reconstr_process(void* classRef)
{
	return ((MyReconstr_Controller *)classRef)->reconstr_process();
}


void MyReconstr_Controller::load_currect_images_and_pose() 
{
	static int last_time = 0;
	
	// (1) load images
	CURRENT_IMAGES = DATABANK.get_current_images(this->SYSTEM_TIME);

	// (2) load cam poses (Note: the order matters!)
	CURRENT_CAM_POSES = DATABANK.get_current_cam_poses(true);

	if(DATABANK.check_data_ending())
		this-> SYSTEM_STATE = SYSTEM_DATA_ENDING;

	// (3) display the data once every second
	bool ready_to_show_on_console =  ((SYSTEM_STATE == SYSTEM_DATA_DISPLAY_MODE) || (SYSTEM_STATE == SYSTEM_RECONSTR_DEBUG_MODE))
				      && (rint(this->SYSTEM_TIME) != last_time);

	if(ready_to_show_on_console)
	{		
		CONSOLE.show_camera_pose(this->SYSTEM_TIME,CURRENT_CAM_POSES);
		last_time = rint(this->SYSTEM_TIME);
	}	
}


void MyReconstr_Controller::clean_up_and_ready_for_restart()
{
	// (1) clean up all the variables ToDo!!
	this->USER_INPUT_PAUSE = false;
	this->USER_INPUT_SHOWSTATUS = false;
	this->NEW_IMAGE_TO_PUB = false;
	this->NEW_MODEL_TO_PUB = false;

	this->SYSTEM_TIME = 0.0;

	// (2) clear images and reconstructed model
	Model3D.clear();
	Images2D = cv::Mat(IMAGE_H, IMAGE_W, CV_8UC3, Scalar(0, 0, 0));
	destroyWindow(OPENCV_IMSHOW_WINDOW_NAME);
	namedWindow(OPENCV_IMSHOW_WINDOW_NAME);

	// (3) clear class object variables
	DATABANK.reset_data_pointers();
	CONSOLE.reset_data_pointers();
}


void MyReconstr_Controller::final_result_publish_and_display()
{
	// (1) wrap up reconstruction result and publish last stuff
	Model3D_Last.points.resize(Model3D.points.size());
	copyPointCloud(Model3D_Last,Model3D);
	Images2D_Last = Images2D.clone();

	imagePb();
	modelPb();

	// (2) handle last console status display
	this->USER_INPUT_SHOWSTATUS = true;
}


void MyReconstr_Controller::imagePb() 
{
	// (1) check if ready to publish	
	if(IMAGE_STATE == IMAGE_DONE_PROCESSING || SYSTEM_STATE == SYSTEM_DATA_ENDING)
	{
		IMAGE_STATE = IMAGE_START_PUBLISHING;

		// (2) send new command
		if(NEW_IMAGE_TO_PUB)
		{
			try
			{	
				//ref: https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros
				cv_bridge::CvImage img_bridge;
				sensor_msgs::Image img_msg;
				std_msgs::Header header; 
				header.seq = IMAGE_PUB_COUNT;    // user defined counter
				header.stamp = ros::Time::now(); // time
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, Images2D);
				img_bridge.toImageMsg(img_msg);  // from cv_bridge to sensor_msgs::Image
				image_pub_.publish(img_msg);
				NEW_IMAGE_TO_PUB = false;
			}
			catch (cv_bridge::Exception& e)
			{
				CONSOLE.display_system_message(5);
				ROS_ERROR("%s",e.what());
				return;
			}
		}
		IMAGE_STATE = IMAGE_DONE_PUBLISHING;
	}

	// (3) update published data count
	IMAGE_PUB_COUNT ++;
}


void MyReconstr_Controller::modelPb() 
{
	// (1) check if ready to publish	
	if(MODEL_STATE == MODEL_DONE_PROCESSING || SYSTEM_STATE == SYSTEM_DATA_ENDING)
	{
		MODEL_STATE = MODEL_START_PUBLISHING;

		// (2) send new ros topic out
		if(NEW_MODEL_TO_PUB)
		{
			try
			{	
				stringstream ss;
				ss << (MODEL_PUB_COUNT+1);
				string s = ss.str();

				model_pub_.publish(Model3D.makeShared());
				NEW_MODEL_TO_PUB = false;
			}
			catch (int e)
			{
				CONSOLE.display_system_message(6);
				return;
			}
		}
		MODEL_STATE = MODEL_DONE_PUBLISHING;
	}

	// (3) update published data count
	MODEL_PUB_COUNT ++;
}

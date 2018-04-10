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

	this->NEW_IMAGE_TO_PUB = false;
	this->NEW_MODEL_TO_PUB = false;
	this->IMAGE_STATE = IMAGE_EMPTY;
	this->MODEL_STATE = MODEL_EMPTY;
	this->SYSTEM_STATE = SYSTEM_JUST_STARTING;

	this->GOODBYE = false;
}


bool MyReconstr_Controller::init_ros(int argc, char** argv)
{
	while(!ros::ok())
		CONSOLE.display_system_message(3);

	// Setup Image publish/subscribe relation
	image_pub_ = it_.advertise(ROS_TOPIC_PUBLISH_NAME1, 1); 
	model_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ROS_TOPIC_PUBLISH_NAME2, 1,true);
	
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
	static ros::Rate loop_rate(CONSOLE_LOOP_RATE);
	while (ros::ok() && !GOODBYE)
  	{
		loop_rate.sleep();
		ros::spinOnce();  
	}
}


void *MyReconstr_Controller::io_process()
{
	static ros::Rate loop_rate(IO_LOOP_RATE);
	while (ros::ok() && !GOODBYE)
  	{
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
		ros::spinOnce();   // ensures that the callback gets called
		loop_rate.sleep();
	}
}


void *MyReconstr_Controller::reconstr_process()
{
	while (ros::ok() && !GOODBYE)
  	{
		
		
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


void MyReconstr_Controller::imagePb() 
{
	// (1) check if ready to publish	
	if(IMAGE_STATE == IMAGE_DONE_PROCESSING)
	{
		IMAGE_STATE = IMAGE_START_PUBLISHING;

		// (2) send new command
		if(NEW_IMAGE_TO_PUB)
		{
			try
			{	
				image_pub_.publish(Images2D->toImageMsg());
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
	if(MODEL_STATE == MODEL_DONE_PROCESSING)
	{
		MODEL_STATE = MODEL_START_PUBLISHING;

		// (2) send new ros topic out
		if(NEW_MODEL_TO_PUB)
		{
			try
			{	stringstream ss;
				ss << (MODEL_PUB_COUNT+1);
				string s = ss.str();

				// ref: http://wiki.ros.org/pcl_ros
				sensor_msgs::PointCloud2::Ptr msg; 
				toROSMsg(Model3D, *msg);
				msg->header.frame_id = s.c_str();

				model_pub_.publish(msg);
				NEW_MODEL_TO_PUB = false;

				throw 6;
			}
			catch (int e)
			{
				CONSOLE.display_system_message(e);
				return;
			}
		}
		MODEL_STATE = MODEL_DONE_PUBLISHING;
	}

	// (3) update published data count
	MODEL_PUB_COUNT ++;
}

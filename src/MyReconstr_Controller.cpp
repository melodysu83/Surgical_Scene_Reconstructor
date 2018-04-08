#include <myproject/MyReconstr_Controller.h>

MyReconstr_Controller::MyReconstr_Controller(int argc, char** argv):it_(nh_)
{
	// load ROS parameters from launch file
	load_ros_param();

	// load the dataset from the csv files
	load_dataset(); 
	
	//init the controller object
	init_sys();
	if(!init_ros(argc,argv))
	{	
		CONSOLE.display_system_message(4);
		exit(1);
	}
}


MyReconstr_Controller::~MyReconstr_Controller
{
}


void MyReconstr_Controller::load_ros_param()
{
	// CAMERA_COUNT: the number of camera viewpoints    
                                                     
	nh_.param("/Surgical_Scene_Reconstructor/CAMERA_COUNT", CAMERA_COUNT, 0);
	CONSOLE.set_camera_count(CAMERA_COUNT);

	if(CAMERA_COUNT<2)
	{
		CONSOLE.display_system_message(0);
		exit(1);
	}
	else
	{
		// IMG_FOLDER: the folders that store images from each camera
		// TME_STAMP: the time stamp for the image sequences for each camera
		// CAM_POSE: the camera pose traversing through time for each camera
		// CAM_CALI: the camera intrinsic and distortion parameters for each camera

		IMG_FOLDER = new string[CAMERA_COUNT];
		TME_STAMP_FILE = new string[CAMERA_COUNT];
		CAM_POSE_FILE = new string[CAMERA_COUNT];
		CAM_CALI_FILE = new string[CAMERA_COUNT];

		for(int i=1; i<=CAMERA_COUNT; i++)
		{
			stringstream ss;
			ss << i;
			string s1 = "/Surgical_Scene_Reconstructor/IMG_FOLDER"+ss.str();
			string s2 = "/Surgical_Scene_Reconstructor/TME_STAMP"+ss.str();
			string s3 = "/Surgical_Scene_Reconstructor/CAM_POSE"+ss.str();
			string s4 = "/Surgical_Scene_Reconstructor/CAM_CALI"+ss.str();

			char* param_name1;
			char* param_name2;
			char* param_name3;
			char* param_name4;

			param_name1 = new char[s1.length() + 1];
	 		param_name2 = new char[s2.length() + 1];
			param_name3 = new char[s3.length() + 1];
			param_name4 = new char[s4.length() + 1];

			strcpy(param_name1, s1.c_str());
			strcpy(param_name2, s2.c_str());
			strcpy(param_name3, s3.c_str());
			strcpy(param_name4, s4.c_str());

			nh_.param(param_name1, IMG_FOLDER[i-1], 0);
			nh_.param(param_name2, TME_STAMP_FILE[i-1], 0);
			nh_.param(param_name3, CAM_POSE_FILE[i-1], 0);
			nh_.param(param_name4, CAM_CALI_FILE[i-1], 0);
		}
		
		CONSOLE.display_system_message(1);
	}
}


void MyReconstr_Controller::load_dataset()
{
	for(int i=1; i<=CAMERA_COUNT; i++)
	{
		//load timestamp
		
		//load campose
		
		//load camera calibration parameters
		
	}
	CONSOLE.display_system_message(2);
}


void MyReconstr_Controller::init_sys()
{
	//set the total number of images from each camera
	IMAGE_COUNT = new int[CAMERA_COUNT];
	//ToDo: add this part, but need to disable for real-time implementation
	//for(int i=1; i<=CAMERA_COUNT; i++)
	//	IMAGE_COUNT[i-1] = ;
	
	//set other system parameters
}


bool MyReconstr_Controller::init_ros(int argc, char** argv)
{
	while(!ros::ok())
		CONSOLE.display_system_message(3);

	// Setup Image publish/subscribe relation
	image_pub_ = it_.advertise("/surgical_scene_reconstr/2D_images", 1); 
	model_pub_ = nh_.advertise("/surgical_scene_reconstr/3D_model", 1);
	
	//ToDo: if allow realtime, need to subscribe to image ros topics

	return true;
		
}


void MyReconstr_Controller::imagePb() 
{
	/*
	// (1) check if ready to publish	
	if(IMAGE_STATUS == DONE_PROCESSING)
	{
		IMAGE_STATUS = START_PUBLISHING;

		// (2) send new command
		if(NEW_IMAGE_TO_PUB)
		{
			try
			{	
				image_pub_.publish(Images2D->toImageMsg());
				NEW_SEGMENTED_IMAGE = false;
			}
			catch (cv_bridge::Exception& e)
			{
				CONSOLE.display_system_message(5);
				ROS_ERROR("%s",e.what());
				return;
			}
		}
		IMAGE_STATUS = DONE_PUBLISHING;
	}

	// (3) update published data count
	IMAGE_PUB_COUNT ++;
	*/
}


void MyReconstr_Controller::modelPb() 
{
	/*
	// (1) check if ready to publish	
	if(MODEL_STATUS == DONE_PROCESSING)
	{
		MODEL_STATUS = START_PUBLISHING;

		// (2) send new ros topic out
		if(NEW_MODEL_TO_PUB)
		{
			try
			{	stringstream ss;
				ss << (MODEL_PUB_COUNT+1);
				string s = ss.str();

				PointCloud::Ptr msg (new PointCloud);
				msg->header.frame_id = s.c_str();
				msg->height = msg->width = 1;
				msg->points = Model3D;
				model_pub_.publish(msg);
				NEW_MODEL_GENERATED = false;
			}
			catch (pcl::PointCloud::Exception& e)
			{
				CONSOLE.display_system_message(5);
				ROS_ERROR("%s",e.what());
				return;
			}
		}
		MODEL_STATUS = DONE_PUBLISHING;
	}

	// (3) update published data count
	MODEL_PUB_COUNT ++;
	*/
}

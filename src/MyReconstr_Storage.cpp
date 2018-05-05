#include "myproject/MyReconstr_Storage.h"

MyReconstr_Storage::MyReconstr_Storage()
{
	this->CAMERA_COUNT = 0;
	this->SYSTEM_TIME = 0;
	this->DATA_END_FLAG = false;
}


MyReconstr_Storage::~MyReconstr_Storage()
{
}


void MyReconstr_Storage::set_camera_count(int cam_cnt)
{
	// CAMERA_COUNT: the number of camera viewpoints    
	this->CAMERA_COUNT = cam_cnt;
	CURRENT_IMAGES_INDEX.assign(cam_cnt,-1);

	CONSOLE.set_camera_count(cam_cnt);

	// IMG_FOLDER: the folders that store images from each camera
	// TME_STAMP: the time stamp for the image sequences for each camera
	// CAM_POSE: the camera pose traversing through time for each camera
	// CAM_CALI: the camera intrinsic and distortion parameters for each camera
	this->IMG_FOLDER = new string[CAMERA_COUNT];
	this->TME_STAMP_FILE = new string[CAMERA_COUNT];
	this->CAM_POSE_FILE = new string[CAMERA_COUNT];
	this->CAM_CALI_FILE = new string[CAMERA_COUNT];
}


void MyReconstr_Storage::set_ros_param_val(int cam_idx, vector<string> param_val)
{
	if(cam_idx > CAMERA_COUNT || cam_idx < 1)
	{
		CONSOLE.display_system_message(8);
		exit(1);
	}
	else if(param_val.size() != 4)
	{
		CONSOLE.display_system_message(9);
		exit(1);
	}
	
	IMG_FOLDER[cam_idx-1] = param_val[0];
	TME_STAMP_FILE[cam_idx-1] = param_val[1];
	CAM_POSE_FILE[cam_idx-1] = param_val[2];
	CAM_CALI_FILE[cam_idx-1] = param_val[3];
}


vector<char*> MyReconstr_Storage::get_ros_param_name(int cam_idx)
{
	if(cam_idx > CAMERA_COUNT || cam_idx < 1)
	{
		CONSOLE.display_system_message(8);
		exit(1);
	}

	stringstream ss;
	ss << cam_idx;
	string s1 = ROS_PARAMETER_NAME1 + ss.str();
	string s2 = ROS_PARAMETER_NAME2 + ss.str();
	string s3 = ROS_PARAMETER_NAME3 + ss.str();
	string s4 = ROS_PARAMETER_NAME4 + ss.str();

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

	vector<char*> param_name;
	param_name.push_back(param_name1);
	param_name.push_back(param_name2);
	param_name.push_back(param_name3);
	param_name.push_back(param_name4);

	return param_name;
}


string MyReconstr_Storage::get_img_folder_name(int cam_idx)
{
	if(cam_idx > CAMERA_COUNT || cam_idx < 1)
	{
		CONSOLE.display_system_message(8);
		exit(1);
	}

	string str = IMG_FOLDER[cam_idx-1];
	return str;
}


void MyReconstr_Storage::load_dataset()
{
	for(int i=1; i<=CAMERA_COUNT; i++)
	{
		// load timestamp
		vector<vector<double> > tme_stamp_data;      
		ifstream tme_stamp_file(TME_STAMP_FILE[i-1].c_str()); 

		if(!tme_stamp_file.good())
			CONSOLE.display_system_message(12);

		while (tme_stamp_file.good())
		{
			vector<double> row; 
			string line;
        		getline(tme_stamp_file, line);
			if ( !tme_stamp_file.good() )
            			break;

			stringstream iss(line);
			for (int col = 0; col < 3; col++) 
			{
				string strval;
				getline(iss, strval,',');
				stringstream convertor(strval);
				double val;
				convertor >> val;
				row.push_back(val); 
			}
			tme_stamp_data.push_back(row);
			
		}
		
		TME_STAMP_DATA.push_back(tme_stamp_data);

		// load campose
		vector<vector<double> > cam_pose_data;      
		ifstream cam_pose_file(CAM_POSE_FILE[i-1].c_str()); 

		if(!cam_pose_file.good())
			CONSOLE.display_system_message(13);

		while (cam_pose_file.good() )
		{
			vector<double> row; 
			string line;
        		getline(cam_pose_file, line);
			if ( !cam_pose_file.good() )
            			break;
			stringstream iss(line);
			for (int col = 0; col < 17; col++) 
			{
				string strval;
				getline(iss, strval,',');
				stringstream convertor(strval);
				double val;
				convertor >> val;
				row.push_back(val); 
			}
			cam_pose_data.push_back(row);
		}
		CAM_POSE_DATA.push_back(cam_pose_data);

		// load camera calibration parameters
		size_t start,end;
		ifstream cam_cali_file(CAM_CALI_FILE[i-1].c_str());// this is good!
		
		if(!cam_cali_file.good())
			CONSOLE.display_system_message(14);

		stringstream buffer;
		string buf,cam_cali_string,data1,data2;
		vector<string> tokens1;
		vector<string> tokens2;

		string target1 = "data: [";
		string target2 = "]\n";

		buffer << cam_cali_file.rdbuf();
		cam_cali_string = buffer.str();

		start = cam_cali_string.find(target1); // this is where intrinsic is
		if (start!=string::npos)
		{
			end = cam_cali_string.find(target2.c_str(),start+1);
			data1 = cam_cali_string.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
		}
		else
			CONSOLE.no_valid_camera_info_file(1,start,end);

		start = cam_cali_string.find(target1.c_str(),end+1); // this is where distortion is
		if (start!=string::npos)
		{
			end = cam_cali_string.find(target2.c_str(),start+1);
			data2 = cam_cali_string.substr(start+7, end-start-7); // string.substr(starting_index, length_of_sub_string)
		}
		else
			CONSOLE.no_valid_camera_info_file(2,start,end);

		stringstream ss1(data1);
		if(cam_cali_file.is_open())
		{	
			while (ss1 >> buf)
				tokens1.push_back(buf.substr(0,buf.length()-1));
		}
		else
			CONSOLE.no_valid_camera_info_file(3,start,end);

		stringstream ss2(data2);
		if(cam_cali_file.is_open())
		{	
			while (ss2 >> buf)
				tokens2.push_back(buf.substr(0,buf.length()-1));
		}
		else
			CONSOLE.no_valid_camera_info_file(3,start,end);

		float distortion_data[5] = { 0,0,0,0,0};
		float intrinsic_data[9] = { 0,0,0,0,0,0,0,0,0};

		if(tokens1.size() == 9)
		{
			stringstream convertor1_0(tokens1[0]);	stringstream convertor1_1(tokens1[1]);	stringstream convertor1_2(tokens1[2]);
			stringstream convertor1_3(tokens1[3]);	stringstream convertor1_4(tokens1[4]);	stringstream convertor1_5(tokens1[5]);
			stringstream convertor1_6(tokens1[6]);	stringstream convertor1_7(tokens1[7]);	stringstream convertor1_8(tokens1[8]);
			convertor1_0 >> intrinsic_data[0];	convertor1_1 >> intrinsic_data[1];	convertor1_2 >> intrinsic_data[2];
			convertor1_3 >> intrinsic_data[3];	convertor1_4 >> intrinsic_data[4];	convertor1_5 >> intrinsic_data[5];
			convertor1_6 >> intrinsic_data[6];	convertor1_7 >> intrinsic_data[7];	convertor1_8 >> intrinsic_data[8];
		}
		if(tokens2.size() == 5)
		{
			stringstream convertor2_0(tokens2[0]);	stringstream convertor2_1(tokens2[1]);	stringstream convertor2_2(tokens2[2]);
			stringstream convertor2_3(tokens2[3]);	stringstream convertor2_4(tokens2[4]);
			convertor2_0 >> distortion_data[0];	convertor2_1 >> distortion_data[1];	convertor2_2 >> distortion_data[2];	
			convertor2_3 >> distortion_data[3];	convertor2_4 >> distortion_data[4];	
		}

		cv::Mat distortion = cv::Mat(1,5,CV_32F,distortion_data);
		cv::Mat intrinsic = cv::Mat(3,3,CV_32F,intrinsic_data);
		
		cv::Mat emptyMat;
		CALI_INTRI_DATA.push_back(emptyMat);
		CALI_DISTO_DATA.push_back(emptyMat);
		CALI_INTRI_DATA[i-1] = intrinsic.clone();
		CALI_DISTO_DATA[i-1] = distortion.clone();		
	}
}


void MyReconstr_Storage::verify_loaded_dataset()
{
	for(int i=1; i<=CAMERA_COUNT; i++)
	{
		// output to console
		CONSOLE.show_csv_data_summary(i,TME_STAMP_DATA[i-1].size(),CAM_POSE_DATA[i-1].size(),CALI_INTRI_DATA[i-1],CALI_DISTO_DATA[i-1]);
	}
	CONSOLE.display_system_message(2);
}


void MyReconstr_Storage::set_system_time(double system_time)
{
	this->SYSTEM_TIME = system_time;
}


bool MyReconstr_Storage::check_data_ending()
{
	bool check = DATA_END_FLAG;
	return check;
}


void MyReconstr_Storage::reset_data_pointers()
{
	
	this->DATA_END_FLAG = false;
	this->SYSTEM_TIME = 0.0;

	CONSOLE.reset_data_pointers();

	for(int i=0; i<CAMERA_COUNT; i++)
		CURRENT_IMAGES_INDEX[i] = -1;
	
}


cv::Mat MyReconstr_Storage::get_random_image()
{
	int cam_choice = rand() % CAMERA_COUNT;
	vector<cv::Mat> imgs = get_random_images();
	cv::Mat img = imgs[cam_choice].clone();
	return img;
}


vector<cv::Mat> MyReconstr_Storage::get_random_images()
{
	bool success = false;
	int out_of_bound_count = 0;
	vector<cv::Mat> current_images;

	while(!success)
	{
		success = true;
		current_images.clear();
		double time_now = double(rand() % 6000)/10.0;
		for(int i=0; i<CAMERA_COUNT; i++)
		{
			int row_max = TME_STAMP_DATA[i].size();
			
			// (1) filter out weird system time
			if(time_now > TME_STAMP_DATA[i][row_max-1][TIME_STAMP_COL])
			{
				success = false;
				break;	
			}

			// (2) find image index
			int idx = image_index_binary_search(time_now,i,0,row_max-1); // cam_idx, row_start,row_end
			if(idx == -1)
			{
				success = false;
				break;
			}
			else
			{
				std::ostringstream ss;
	    			ss << std::setw(IMAGE_FILE_DIGITS) << std::setfill( '0' ) << idx;
				cv::Mat img_curr = imread(get_img_folder_name(i+1)+ss.str()+".png");
				current_images.push_back(img_curr.clone());
			}
		}
	}
	return current_images;
}


vector<cv::Mat> MyReconstr_Storage::get_current_images()
{
	int out_of_bound_count = 0;
	vector<cv::Mat> current_images;
	double time_now = (this->SYSTEM_TIME);

	for(int i=0; i<CAMERA_COUNT; i++)
	{
		int row = CURRENT_IMAGES_INDEX[i];
		int row_max = TME_STAMP_DATA[i].size();

		// (1) filter out weird system time
		if(time_now < 0)
		{
			CONSOLE.get_data_error(5); // Invalid time range
			break;
			exit(1);
		}
		else if(time_now > TME_STAMP_DATA[i][row_max-1][TIME_STAMP_COL])
		{
			CONSOLE.get_data_error(6); // Running out of time
			this->DATA_END_FLAG = true;
			break;	
		}

		// (2) find image index
		if(row == -1) // Case 1: first entry. Need to do binary search.
		{
			row = current_image_index_binary_search(i,0,row_max-1); // cam_idx, row_start,row_end
		}
		else          // Case 2: Start searching from previous index.
		{
			if((row+1 < row_max) && (time_now <= TME_STAMP_DATA[i][row+1][TIME_STAMP_COL]))
			{
				row = current_image_index_find_closest(i,row);
			}
			else if((row+2 < row_max) && (time_now <= TME_STAMP_DATA[i][row+2][TIME_STAMP_COL]))
			{
				row = current_image_index_find_closest(i,row+1);
			}
			else
			{
				row = current_image_index_binary_search(i,row,row_max-1); 
			}
		}
		CURRENT_IMAGES_INDEX[i] = row;

		if(CURRENT_IMAGES_INDEX[i] == -1)
		{
			CONSOLE.get_data_error(7);  
			cv::Mat img_curr = Mat(IMAGE_H,IMAGE_W, CV_64F, double(0));
			current_images.push_back(img_curr.clone());
			out_of_bound_count = out_of_bound_count+1;			
		}
		else
		{
			std::ostringstream ss;
    			ss << std::setw(IMAGE_FILE_DIGITS) << std::setfill( '0' ) << CURRENT_IMAGES_INDEX[i];
			cv::Mat img_curr = imread(get_img_folder_name(i+1)+ss.str()+".png");
			current_images.push_back(img_curr.clone());
		}
	}

	if(out_of_bound_count == CAMERA_COUNT)
	{
		CONSOLE.get_data_error(8);  // all images are not in valid time range.
		exit(1);
	}

	if(current_images.size() != this->CAMERA_COUNT)
		CONSOLE.get_data_error(9);

	return current_images;
}


vector<cv::Mat> MyReconstr_Storage::get_current_images(double system_time)
{
	set_system_time(system_time);
	vector<cv::Mat>  current_images = get_current_images();
	return current_images;
}


vector<vector<double> > MyReconstr_Storage::get_current_cam_poses(bool USE_IMAGE_CAPTURED_TIME)
{
	vector<double> specified_time;
	vector<vector<double> > camera_poses(CAMERA_COUNT,vector<double>(CAM_POSE_SIZE));
	
	if(USE_IMAGE_CAPTURED_TIME)
	{	
		for(int i=0; i<CAMERA_COUNT;i++)
			specified_time.push_back(TME_STAMP_DATA[i][CURRENT_IMAGES_INDEX[i]][TIME_STAMP_COL]);

		camera_poses = get_current_cam_poses(specified_time);
	}
	else
	{
		specified_time.assign(CAMERA_COUNT,this->SYSTEM_TIME);
		camera_poses = get_current_cam_poses(specified_time);
	}
	return camera_poses;
}


vector<vector<double> > MyReconstr_Storage::get_current_cam_poses(vector<double> specified_time)
{
	int out_of_bound_count = 0;
	vector<vector<double> > camera_poses(CAMERA_COUNT,vector<double>(CAM_POSE_SIZE));

	if(specified_time.size() != CAMERA_COUNT)
	{
		CONSOLE.display_system_message(10);
		exit(1);
	}

	for(int i=0; i<CAMERA_COUNT; i++)
	{
		int row = rint(specified_time[i]/TIME_STEP_SIZE);
		if(row<0 || row>=CAM_POSE_DATA[i].size())
		{
			CONSOLE.get_data_error(1);  
			camera_poses[i].assign(CAM_POSE_SIZE,0.0);  // 16 doubles with a value of 0.0
			out_of_bound_count = out_of_bound_count+1;
		}
		else
		{
			if(CAM_POSE_DATA[i][row].size() == CAM_POSE_SIZE+1)
			{
				for(int col=0; col<CAM_POSE_SIZE; col++)
					camera_poses[i][col] = CAM_POSE_DATA[i][row][col+1];
			}
			else
			{
				CONSOLE.get_data_error(2); // something wrong with data
				exit(1);
			}
		}
	}

	if(out_of_bound_count == CAMERA_COUNT)
	{	
		CONSOLE.get_data_error(0);  // all camera poses are not in valid time range.
		exit(1);
	}

	return camera_poses;
}


vector<cv::Mat> MyReconstr_Storage::get_intrinsic_matrices()
{
	vector<cv::Mat> result = CALI_INTRI_DATA;
	return result;
}


vector<cv::Mat> MyReconstr_Storage::get_distortion_matrices()
{
	vector<cv::Mat> result = CALI_DISTO_DATA;
	return result;
}


int MyReconstr_Storage::current_image_index_binary_search(int cam_idx,int row_start,int row_end)
{
	int row;
	int row_test;
	double time_now = (this->SYSTEM_TIME);

	while( row_end-row_start > 1)
	{
		row_test = rint(0.5*(row_start+row_end));
		if(TME_STAMP_DATA[cam_idx][row_test][TIME_STAMP_COL] > time_now)
			row_end = row_test;
		else
			row_start = row_test;
	}
	
	row = current_image_index_find_closest(cam_idx,row_start);
	return row;
}


int MyReconstr_Storage::current_image_index_find_closest(int cam_idx,int row_prev)
{
	int row;
	int row_max = TME_STAMP_DATA[cam_idx].size();
	double time_now = (this->SYSTEM_TIME);

	if(row_prev<row_max && row_prev+1<row_max)
	{
		double time_prev = TME_STAMP_DATA[cam_idx][row_prev][TIME_STAMP_COL];
		double time_next = TME_STAMP_DATA[cam_idx][row_prev+1][TIME_STAMP_COL];

		if(time_now-time_prev > time_next-time_now || time_prev < 0)
			row = row_prev+1;
		else
			row = row_prev;
	}
	else if(row_prev>=row_max)
	{
		row = -1;
		CONSOLE.get_data_error(3);
		exit(1);
	}
	else
	{
		row = row_prev;
		CONSOLE.get_data_error(4);
	}

	return row;
}


int MyReconstr_Storage::image_index_binary_search(double time_now, int cam_idx,int row_start,int row_end)
{
	int row;
	int row_test;

	while( row_end-row_start > 1)
	{
		row_test = rint(0.5*(row_start+row_end));
		if(TME_STAMP_DATA[cam_idx][row_test][TIME_STAMP_COL] > time_now)
			row_end = row_test;
		else
			row_start = row_test;
	}
	
	row = image_index_find_closest(time_now,cam_idx,row_start);

	return row;
}


int MyReconstr_Storage::image_index_find_closest(double time_now, int cam_idx,int row_prev)
{
	int row;
	int row_max = TME_STAMP_DATA[cam_idx].size();

	if(row_prev<row_max && row_prev+1<row_max)
	{
		double time_prev = TME_STAMP_DATA[cam_idx][row_prev][TIME_STAMP_COL];
		double time_next = TME_STAMP_DATA[cam_idx][row_prev+1][TIME_STAMP_COL];

		if(time_now-time_prev > time_next-time_now || time_prev < 0)
			row = row_prev+1;
		else
			row = row_prev;
	}
	else if(row_prev>=row_max)
	{
		row = -1;
		CONSOLE.get_data_error(3);
		exit(1);
	}
	else
	{
		row = row_prev;
		CONSOLE.get_data_error(4);
	}

	return row;
}


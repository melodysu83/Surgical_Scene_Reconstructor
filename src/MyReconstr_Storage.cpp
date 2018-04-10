#include "myproject/MyReconstr_Storage.h"

MyReconstr_Storage::MyReconstr_Storage()
{
	CAMERA_COUNT = 0;
}


MyReconstr_Storage::~MyReconstr_Storage()
{
}


void MyReconstr_Storage::set_camera_count(int cam_cnt)
{
	// CAMERA_COUNT: the number of camera viewpoints    
	this->CAMERA_COUNT = cam_cnt;
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


void MyReconstr_Storage::load_dataset()
{
	for(int i=1; i<=CAMERA_COUNT; i++)
	{
		// load timestamp
		vector<vector<double> > tme_stamp_data;      
		ifstream tme_stamp_file(TME_STAMP_FILE[i-1].c_str()); 
		
		while (tme_stamp_file.good() )
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




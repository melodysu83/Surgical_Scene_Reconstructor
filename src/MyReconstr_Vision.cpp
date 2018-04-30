#include "myproject/MyReconstr_Vision.h"

MyReconstr_Vision::MyReconstr_Vision()
{
	this->MY_FEATURE_ALGO = DEFAULT_FEATURE_ALGO;

	this->WIN_PYR = cv::Size(FEATURE_TRACKING_PYR_WIN_SIZE,FEATURE_TRACKING_PYR_WIN_SIZE);
	reset_feature_parameters(); // set feature detection parameter values
	reset_processing();	    // reset memlory release flag
}


MyReconstr_Vision::~MyReconstr_Vision()
{
}


void MyReconstr_Vision::load_intrinsic_matrices(vector<cv::Mat> data)
{
	this->CALI_INTRI_DATA = data;
}


void MyReconstr_Vision::load_distortion_matrices(vector<cv::Mat> data)
{
	this->CALI_DISTO_DATA = data;
}


void MyReconstr_Vision::reset_feature_parameters()
{
	// for FAST_ALGO
	this->FEATURE_PARAM_INTENSITY_THRES = DEFAULT_FEATURE_PARAM_INTENSITY_THRES;
	this->FEATURE_PARAM_NON_MAX_SUPPRE = DEFAULT_FEATURE_PARAM_NON_MAX_SUPPRE;	

	// for SURF_ALGO
	this->FEATURE_PARAM_HESSIAN_THRES = DEFAULT_FEATURE_PARAM_HESSIAN_THRES;
	this->FEATURE_PARAM_N_PYRAMIDS = DEFAULT_FEATURE_PARAM_N_PYRAMIDS;
	this->FEATURE_PARAM_N_PYRAMID_LAYERS = DEFAULT_FEATURE_PARAM_N_PYRAMID_LAYERS;
	this->FEATURE_PARAM_DESCRIPTOR_EXTENDED = DEFAULT_FEATURE_PARAM_DESCRIPTOR_EXTENDED;

	// for ORB_ALGO
	this->FEATURE_PARAM_N_FEATURES = DEFAULT_FEATURE_PARAM_N_FEATURES;
	this->FEATURE_PARAM_SCALE = DEFAULT_FEATURE_PARAM_SCALE;
	this->FEATURE_PARAM_N_LEVELS = DEFAULT_FEATURE_PARAM_N_LEVELS;
	this->FEATURE_PARAM_EDGE_THRES = DEFAULT_FEATURE_PARAM_EDGE_THRES;

	// initialize detectors
	create_feature_detectors();

	// clear feature points
	features_inter_cam.clear();
	features_intra_cam.clear();
}


void MyReconstr_Vision::reset_processing()
{
	this->MEMORY_RELEASE_FLAG = true;
}


void MyReconstr_Vision::set_feature_parameters_fast(int param1,bool param2)
{
	this->FEATURE_PARAM_INTENSITY_THRES = param1;
	this->FEATURE_PARAM_NON_MAX_SUPPRE = param2;

	create_feature_detectors();
}


void MyReconstr_Vision::set_feature_parameters_surf(double param1,int param2,int param3,bool param4)
{
	this->FEATURE_PARAM_HESSIAN_THRES = param1;
	this->FEATURE_PARAM_N_PYRAMIDS = param2;
	this->FEATURE_PARAM_N_PYRAMID_LAYERS = param3;
	this->FEATURE_PARAM_DESCRIPTOR_EXTENDED = param4;
	
	create_feature_detectors();
}


void MyReconstr_Vision::set_feature_parameters_orb(int param1,float param2,int param3,int param4)
{
	this->FEATURE_PARAM_N_FEATURES = param1;
	this->FEATURE_PARAM_SCALE = param2;
	this->FEATURE_PARAM_N_LEVELS = param3;
	this->FEATURE_PARAM_EDGE_THRES = param4;

	create_feature_detectors();
}


void MyReconstr_Vision::show_feature_parameters(FEATURE_ALGO_LIST my_algo)
{
	switch(my_algo)
	{
		case FAST_ALGO:
			CONSOLE.show_feature_detection_parameters_fast(FEATURE_PARAM_INTENSITY_THRES,FEATURE_PARAM_NON_MAX_SUPPRE);
			break;

		case SURF_ALGO:
			CONSOLE.show_feature_detection_parameters_surf(FEATURE_PARAM_HESSIAN_THRES,FEATURE_PARAM_N_PYRAMIDS,FEATURE_PARAM_N_PYRAMID_LAYERS,FEATURE_PARAM_DESCRIPTOR_EXTENDED);
			break;	

		case ORB_ALGO:
			CONSOLE.show_feature_detection_parameters_orb(FEATURE_PARAM_N_FEATURES,FEATURE_PARAM_SCALE,FEATURE_PARAM_N_LEVELS,FEATURE_PARAM_EDGE_THRES);
			break;

	}
}


void MyReconstr_Vision::show_all_feature_parameters()
{
	for(int i=0; i<NUM_OF_FEATURE_ALGO; i++)
	{
		show_feature_parameters((FEATURE_ALGO_LIST)i);
	}
}


void MyReconstr_Vision::create_feature_detectors()
{
	// ref: https://docs.opencv.org/3.2.0/df/d74/classcv_1_1FastFeatureDetector.html
	// ref: https://docs.opencv.org/3.4.1/d5/df7/classcv_1_1xfeatures2d_1_1SURF.html
	// ref: https://docs.opencv.org/3.4.0/db/d95/classcv_1_1ORB.html

	detector_fast = FastFeatureDetector::create(FEATURE_PARAM_INTENSITY_THRES,FEATURE_PARAM_NON_MAX_SUPPRE);
	detector_surf = xfeatures2d::SURF::create(FEATURE_PARAM_HESSIAN_THRES,FEATURE_PARAM_N_PYRAMIDS,FEATURE_PARAM_N_PYRAMID_LAYERS,this->FEATURE_PARAM_DESCRIPTOR_EXTENDED);
	detector_orb = ORB::create(FEATURE_PARAM_N_FEATURES,FEATURE_PARAM_SCALE,FEATURE_PARAM_N_LEVELS,FEATURE_PARAM_EDGE_THRES); 
}


void MyReconstr_Vision::set_feature_detection_algo(FEATURE_ALGO_LIST my_algo)
{
	this->MY_FEATURE_ALGO = my_algo;
	CONSOLE.set_feature_detection_algo(my_algo);
}


void MyReconstr_Vision::display_all_feature_algos(cv::Mat src)
{
	cv::Mat img;
	vector<cv::Mat> src_vec(NUM_OF_FEATURE_ALGO);
	for(int i=0; i<NUM_OF_FEATURE_ALGO; i++)
	{
		string ss = FEATURE_ALGO_TO_STRING((FEATURE_ALGO_LIST)i);
		Time t_start = ros::Time::now();
		vector<KeyPoint> kpts = feature_extraction(src,(FEATURE_ALGO_LIST)i);
		Time t_end = ros::Time::now();
		CONSOLE.show_runtime(ss,t_end-t_start);
		img = draw_feature_points(src,kpts);
		putText(img, ss, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_COLOR_BLACK, 1, CV_AA);
		src_vec[i] = img.clone();
		
	}
	img = image_collage_maker(src_vec,false);
	imshow(OPENCV_IMSHOW_WINDOW_NAME,img);
	cv::waitKey(30);
}


vector<KeyPoint>  MyReconstr_Vision::feature_extraction(cv::Mat img, FEATURE_ALGO_LIST my_algo)
{
	vector<KeyPoint> kpts;

	switch(my_algo)
	{
		case FAST_ALGO:
			detector_fast->detect(img,kpts,Mat());
			break;

		case SURF_ALGO:
			detector_surf->detect(img,kpts);
			break;	

		case ORB_ALGO:
			detector_orb->detect(img,kpts,Mat());	
			break;

	}
	return kpts;
}


vector<KeyPoint>  MyReconstr_Vision::feature_extraction(cv::Mat img)
{
	vector<KeyPoint> kpts = feature_extraction(img,this->MY_FEATURE_ALGO);
	return kpts;
}


vector<vector<KeyPoint> >  MyReconstr_Vision::feature_extraction_for_images(vector<cv::Mat> img_vec)
{
	vector<vector<KeyPoint> > kpts_vec = feature_extraction_for_images(img_vec,this->MY_FEATURE_ALGO);
	return kpts_vec;
}


vector<vector<KeyPoint> >  MyReconstr_Vision::feature_extraction_for_images(vector<cv::Mat> img_vec, FEATURE_ALGO_LIST my_algo)
{
	int img_cnt = img_vec.size();
	vector<vector<KeyPoint> >  kpts_vec(img_cnt);

	switch(my_algo)
	{
		case FAST_ALGO:
			detector_fast->detect(img_vec,kpts_vec,Mat());
			break;

		case SURF_ALGO:
			detector_surf->detect(img_vec,kpts_vec);
			break;

		case ORB_ALGO:
			detector_orb->detect(img_vec,kpts_vec,Mat());
			break;
	}
	
	return kpts_vec;
}


cv::Mat MyReconstr_Vision::draw_feature_points(cv::Mat src,vector<KeyPoint> kpts)
{
	cv::Mat img;
	drawKeypoints(src, kpts, img);
	return img;
}


cv::Mat MyReconstr_Vision::draw_feature_intra_cam_matches(cv::Mat src,vector<Point2f> pts_old,vector<Point2f> pts_new)
{
	// Goal: use arrows to show feature point motions over time
	cv::Mat img = src.clone();
	
	if(pts_old.size() == pts_new.size())
	{
		int pts_cnt = pts_new.size();
		for(int i=0; i<pts_cnt; i++)
		{
			circle(img, pts_new[i], CV_DRAW_RADIUS, CV_COLOR_WHITE, CV_DRAW_THICKNESS);  
			if(pts_old[i].x != -1)
			{
				circle(img, pts_old[i], CV_DRAW_RADIUS, CV_COLOR_GRAY, CV_DRAW_THICKNESS);  
				line(img, pts_old[i], pts_new[i], CV_COLOR_GRAY, CV_DRAW_THICKNESS, CV_DRAW_TYPE, CV_DRAW_SHIFT); 
			}
		}
	}
	else
		CONSOLE.visual_processing_error(10);
	return img;
}


cv::Mat MyReconstr_Vision::draw_feature_inter_cam_matches(cv::Mat src1,cv::Mat src2,vector<Point2f> pts1,vector<Point2f> pts2)
{
	// Goal: use colors to show feature point matches between cams
	cv::Mat img;
	hconcat(src1,src2,img);
	if(pts1.size() == pts2.size())
	{
		int pts_cnt = pts1.size();
		for(int i=0; i<pts_cnt; i++)
		{
			Point2f pt1 = pts1[i];
			Point2f pt2 = pts2[i]+Point2f(0.0f+src1.cols,0.0f);
			circle(img, pt1, CV_DRAW_RADIUS, CV_COLOR_BLUE, CV_DRAW_THICKNESS);  
			if(pts2[i].y != -1)
			{
				line(img, pt1,pt2, CV_COLOR_RED, CV_DRAW_THICKNESS, CV_DRAW_TYPE, CV_DRAW_SHIFT); 
				circle(img, pt2, CV_DRAW_RADIUS, CV_COLOR_BLUE, CV_DRAW_THICKNESS);  
			}
		}
	}
	else
		CONSOLE.visual_processing_error(10);
	return img;
}


vector<cv::Mat> MyReconstr_Vision::draw_feature_points_for_images(vector<cv::Mat> src_vec,vector<vector<KeyPoint> > kpts_vec)
{
	vector<cv::Mat> img_vec;
	int img_cnt = src_vec.size();

	if(src_vec.size() != kpts_vec.size())
	{
		CONSOLE.visual_processing_error(0);
		exit(1);
	}
	else
	{
		for(int i=0; i<img_cnt; i++)
		{
			cv::Mat img = draw_feature_points(src_vec[i],kpts_vec[i]);
			img_vec.push_back(img.clone());
		}
	}
	
	return img_vec;
}


vector<cv::Mat> MyReconstr_Vision::draw_feature_intra_cam_matches_for_images(vector<cv::Mat> src_vec,vector<vector<Point2f> > features_raw)
{
	vector<cv::Mat> img_vec = src_vec;
	if(src_vec.size() != features_intra_cam.size() || src_vec.size() != features_raw.size())
	{
		CONSOLE.visual_processing_error(7);
		return img_vec;		
	}

	for(int i=0; i<src_vec.size(); i++)
	{
		cv::Mat img = draw_feature_intra_cam_matches(src_vec[i],features_intra_cam[i],features_raw[i]);
		img_vec[i] = img.clone();
		stringstream s;
		s << (i);
		string ss = DRAW_FEATURE_INTRA_CAM_TEXT+s.str();
		putText(img_vec[i], ss, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_COLOR_BLACK, 1, CV_AA);
	}
	
	return img_vec;
}


vector<cv::Mat> MyReconstr_Vision::draw_feature_inter_cam_matches_for_images(vector<cv::Mat> src_vec,vector<vector<Point2f> > features_raw,vector<vector<int> > cam_matches_comb)
{
	vector<cv::Mat> img_vec(cam_matches_comb.size());
	bool cant_draw = false;
	if(cam_matches_comb.size() != features_inter_cam.size())
	{
		CONSOLE.visual_processing_error(8);
		cant_draw = true;	
	}
	else if(src_vec.size() != features_raw.size())
	{
		CONSOLE.visual_processing_error(9);
		cant_draw = true;
	}	

	for(int i=0; i<cam_matches_comb.size();i++)
	{
		int idx1 = cam_matches_comb[i][0];
		int idx2 = cam_matches_comb[i][1];
		if(cant_draw)
			hconcat(src_vec[idx1],src_vec[idx2],img_vec[i]);
		else
		{
			cv::Mat img = draw_feature_inter_cam_matches(src_vec[idx1],src_vec[idx2],features_raw[idx1],features_inter_cam[i]);
			img_vec[i] = img.clone();
		}	
		stringstream s1,s2;
		s1 << (idx1+1);
		s2 << (idx2+1);
		string ss = DRAW_FEATURE_INTER_CAM_TEXT+s1.str()+","+s2.str();
		putText(img_vec[i], ss, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_COLOR_BLACK, 1, CV_AA);		
	}	
	int x,y;
	
	vector<cv::Mat> img_vec_double(img_vec.size()*2);
	for(int i=0; i<img_vec.size();i++)
	{
		int sub_img_cols = img_vec[i].cols/2;
		int sub_img_rows = img_vec[i].rows;
		x=0;
		y=0;
		img_vec_double[2*i] = img_vec[i](cv::Rect(x,y, sub_img_cols, sub_img_rows)).clone();

		x=img_vec[i].cols-1-sub_img_cols;
		y=0;
		img_vec_double[2*i+1] = img_vec[i](cv::Rect(x,y, sub_img_cols, sub_img_rows)).clone();
	}
	return img_vec_double;
}


cv::Mat MyReconstr_Vision::image_collage_maker(vector<cv::Mat> src_vec,bool require_even_cols)
{
	int img_cnt = src_vec.size();
	int rows = floor(sqrt(img_cnt));
	int cols = rows;

	if(require_even_cols && cols%2!=0)
		cols = cols+1;
	
	cv::Mat img;

	while(cols*rows<img_cnt)
	{
		if(require_even_cols)
			cols = cols+2;
		else
			cols = cols+1;
	}

	vector<cv::Mat> img_vec(cols*rows);
	double img_scale = double(1.0*OPENCV_IMSHOW_WINDOW_SCALE/cols);

	for(int i=1; i<img_cnt; i++)
	{
		if(src_vec[i].rows != src_vec[0].rows || src_vec[i].cols != src_vec[0].cols)
		{
			CONSOLE.visual_processing_error(1);
			exit(1);
		}
	}

	for(int i=0; i<cols*rows; i++)
	{
		if(i<img_cnt)
			resize(src_vec[i], img_vec[i], cv::Size(), img_scale, img_scale);
		else
			img_vec[i] = cv::Mat(img_vec[i-1].rows,img_vec[i-1].cols,img_vec[i-1].type(), Scalar(0, 0, 0));
	}

	for(int r=0; r<rows; r++)
	for(int c=1; c<cols; c++)
		hconcat(img_vec[r*cols],img_vec[r*cols+c],img_vec[r*cols]);

	for(int r=1; r<rows; r++)
		vconcat(img_vec[0],img_vec[r*cols],img_vec[0]);

	img = img_vec[0].clone();

	return img;
}


cv::Mat MyReconstr_Vision::process_image2D(vector<cv::Mat> curr_imgs,vector<vector<double> > curr_poses, vector<int> camera_group_labels) 
{
	cv::Mat image2D;
	features_inter_cam.clear();
	features_intra_cam.clear();

	if(camera_group_labels.size() != curr_imgs.size() || curr_poses.size() != curr_imgs.size())
	{
		CONSOLE.visual_processing_error(2);
		return image2D;
		
	}
	for(int i=0; i<curr_imgs.size(); i++)
	if(curr_poses[i].size() != CAM_POSE_SIZE)
	{
		CONSOLE.visual_processing_error(3);
		return image2D;
	}

	// (1) prep for required data 
	//     part1: compute camera matches combinations
	vector<vector<int> > camera_matches_comb = IMGFUNC.constrained_combination(camera_group_labels,false); // set: true for debugging

	//     part2: get projection matrix from cam pose data
	vector<cv::Mat> extrinsics = IMGFUNC.poses_to_projMats(curr_poses,true); // of dimension 3x4
	
	//     part3: get optic flow pyramid
	vector<vector<Mat> > imgs_pyr(curr_imgs.size());
	for(int i=0; i<curr_imgs.size(); i++)
		cv::buildOpticalFlowPyramid( curr_imgs[i], imgs_pyr[i], WIN_PYR, FEATURE_TRACKING_PYR_MAX_LAYER); //ToDo: tune these parameters

	// (2) feature point extraction
	vector<vector<KeyPoint> >  kpts_vec = feature_extraction_for_images(curr_imgs);
	vector<vector<Point2f> >   features_raw(kpts_vec.size());
	for(int i=0; i<kpts_vec.size(); i++)
		cv::KeyPoint::convert(kpts_vec[i],features_raw[i],vector<int>());
	
	// (3) feature point tracking (inter-camera: across cameras, no memory)
	features_inter_cam = feature_tracking_inter_camera(features_raw,imgs_pyr,extrinsics,camera_matches_comb); 

	// (4) feature point tracking (intra-camera: across time, memory dependent)
	features_intra_cam = feature_tracking_intra_camera(features_raw,imgs_pyr,extrinsics); // ToDo: think about how to combine (3) and (4)
	// ToDo: we are here <---
	// a. combine intra inter results
	// b. make parameters for matching tunable

	// (5) manage display 
	vector<cv::Mat> img_vec = draw_feature_points_for_images(curr_imgs,kpts_vec);     			            // for (1): print out feature points
	vector<cv::Mat> img_vec_intra = draw_feature_intra_cam_matches_for_images(img_vec,features_raw);   	            // for (4): draw feature point motion over time
	vector<cv::Mat> img_vec_inter = draw_feature_inter_cam_matches_for_images(img_vec,features_raw,camera_matches_comb);// for (3): feature point matches across images
	
	vector<cv::Mat> img_vec_all = img_vec_inter;
	img_vec_all.insert( img_vec_all.end(), img_vec_intra.begin(), img_vec_intra.end() );
	image2D = image_collage_maker(img_vec_all,true); // merging images


	// ToDo: think about addressing camera pose uncertainty ... in (4) and (3)


	// [Other ideas and notes:]
	// - Can look up DescriptorExtractor.
	// - Do the following to access each element of image:
	//   int r=10; int c=24; int b=1; // BGR
	//   Images2D.at<cv::Vec3b>(r,c)[b] = 27;

	return image2D;
}


PointCloud<PointXYZRGB> MyReconstr_Vision::process_model3D() // Next ToDo~
{
	PointCloud<PointXYZRGB> model3D;
	model3D.clear(); //

	// (6) feature point registration (associate feature points IDs) - need to design a data structure for the 2D 3D point relation
	// ToDo: what about accumulative error in camera pose? 
	// when to modify the slow drift from true cam pose? in (6)? 
	// when intra cam tracks are right but majority of points are classified as dynamic	

	// (7) feature point classification (distinguish static and dynamic points)
	 
	// (8) map building --> could be put to process_model3D() function


	// ToDo: Replace the following baby script with real stuff!
	PointXYZRGB pt1, pt2;
	pt1.x = 1;	pt1.y = 2;	pt1.z = 50;	pt1.r = 100;	pt1.g = 100;	pt1.b = 125;
	pt2.x = 1;	pt2.y = 57;	pt2.z = 5;  	pt2.r = 223;	pt2.g = 12;	pt2.b = 50;
	model3D.points.push_back(pt1);
	model3D.points.push_back(pt2);

	// [Other ideas and notes:]
	// - More info about point cloud: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html
	// - what does model3D.clear() do? 
	//   Removes all points in a cloud and sets the width and height to 0.
	// - Do the following to access each element of 3D model:
	//   int pt_idx = 2;
	//   Model3D.points[pt_idx].x , Model3D.points[pt_idx].y , Model3D.points[pt_idx].z
	//   Model3D.points[pt_idx].r , Model3D.points[pt_idx].g , Model3D.points[pt_idx].b

	return model3D;
}


vector<vector<Point2f> > MyReconstr_Vision::feature_tracking_cross_camera(vector<vector<Point2f> > kpts, vector<vector<cv::Mat> > imgs_pyr,vector<cv::Mat> exMat,vector<cv::Mat> inMat,vector<cv::Mat> disMat,vector<vector<int> > cam_comb_idx)
{
	// Goal: feature point tracking (inter-camera: across cameras, no memory)
	//       use camera combo information to find feature point matches
	// ref: https://github.com/Tetragramm/opencv_contrib/blob/master/modules/mapping3d/samples/computeMapping3d.cpp

	vector<vector<Point2f> > kpts_cross_cam(cam_comb_idx.size());

	for(int i=0; i<cam_comb_idx.size(); i++)
	{
		int cam1 = cam_comb_idx[i][0];
		int cam2 = cam_comb_idx[i][1];
		vector<Point2f> kpts_cam2;
		Mat status, err;
		
		if(kpts[cam1].size() > 0)
		{
			cv::calcOpticalFlowPyrLK( imgs_pyr[cam1], imgs_pyr[cam2], kpts[cam1], kpts_cam2, status, err, WIN_PYR, FEATURE_TRACKING_PYR_MAX_LAYER);

			if(kpts[cam1].size() != kpts_cam2.size())
				CONSOLE.visual_processing_error(6);

			for(int j=0; j<kpts[cam1].size();j++)
			{
				if(status.at<uchar>(j) != 1)
				{
					kpts_cam2[j].x = -1;
					kpts_cam2[j].y = -1;
				}
			}
		}
		kpts_cross_cam[i] = kpts_cam2;
	}
	return kpts_cross_cam;
}


vector<vector<Point2f> > MyReconstr_Vision::feature_tracking_inter_camera(vector<vector<Point2f> > kpts_vec, vector<vector<cv::Mat> > imgs_pyr, vector<cv::Mat> extrinsics,vector<vector<int> > cam_comb_idx)
{
	vector<vector<Point2f> > kpts_inter_cam;
	vector<cv::Mat> intrinsics = CALI_INTRI_DATA;
	vector<cv::Mat> distortions = CALI_DISTO_DATA;

	kpts_inter_cam = feature_tracking_cross_camera(kpts_vec,imgs_pyr,extrinsics,intrinsics,distortions,cam_comb_idx);
	return kpts_inter_cam;
	
}


vector<vector<Point2f> > MyReconstr_Vision::feature_tracking_intra_camera(vector<vector<Point2f> > kpts_vec, vector<vector<cv::Mat> > imgs_pyr,vector<cv::Mat> extrinsics)
{
	// (0) variable declarations
	static int count = 0;
	static queue<vector<vector<Point2f> > > old_kpts_vec_stack;
	static queue<vector<vector<cv::Mat> > > old_imgs_pyr_stack;
	static queue<vector<cv::Mat> > old_extrinsics_stack;
	int img_count = imgs_pyr.size();
	int all_cameras_have_feature_points = 1;
	vector<vector<Point2f> > kpts_intra_cam(img_count);

	// (1) find tracking feature points from successive image
	if(!MEMORY_RELEASE_FLAG)
	{
		if(old_kpts_vec_stack.empty() || old_imgs_pyr_stack.empty() || old_extrinsics_stack.empty())
		{
			CONSOLE.visual_processing_error(4);
			return kpts_intra_cam;
		}

		vector<vector<Point2f> > old_kpts_vec = old_kpts_vec_stack.front();
		vector<vector<cv::Mat> > old_imgs_pyr = old_imgs_pyr_stack.front();
		vector<cv::Mat> old_extrinsics = old_extrinsics_stack.front();

		old_kpts_vec_stack.pop();
		old_imgs_pyr_stack.pop();
		old_extrinsics_stack.pop();

		if(old_imgs_pyr.size()!=img_count || old_extrinsics.size()!=img_count || extrinsics.size()!=img_count)
		{
			CONSOLE.visual_processing_error(5);
			return kpts_intra_cam;
		}

		vector<vector<int> > cam_comb_idx(img_count);
		vector<cv::Mat> intrinsics(img_count*2);
		vector<cv::Mat> distortions(img_count*2);
		for(int i=0; i<img_count;i++)
		{
			cam_comb_idx[i].push_back(i);
			cam_comb_idx[i].push_back(i+img_count);

			intrinsics[i] = CALI_INTRI_DATA[i];
			intrinsics[i+img_count] = CALI_INTRI_DATA[i];
			distortions[i] = CALI_DISTO_DATA[i];
			distortions[i+img_count] = CALI_DISTO_DATA[i];

		}

		vector<vector<Point2f> > all_kpts_vec = kpts_vec;
		vector<vector<cv::Mat> > all_imgs_pyr = imgs_pyr;
		vector<cv::Mat> all_extrinsics = extrinsics;

		all_kpts_vec.insert( all_kpts_vec.end(), old_kpts_vec.begin(), old_kpts_vec.end() );
		all_imgs_pyr.insert(all_imgs_pyr.end(), old_imgs_pyr.begin(), old_imgs_pyr.end() );
		all_extrinsics.insert(all_extrinsics.end(), old_extrinsics.begin(), old_extrinsics.end() );

		kpts_intra_cam = feature_tracking_cross_camera(all_kpts_vec,all_imgs_pyr,all_extrinsics,intrinsics,distortions,cam_comb_idx);
		count = count-1;
	}
	else
	{
		for(int i=0; i<img_count;i++)
		{
			vector<Point2f> tmp(kpts_vec[i].size());
			for(int j=0;j<tmp.size();j++)
			{
				tmp[j].x = -1;
				tmp[j].y = -1;
			}
			kpts_intra_cam[i] = tmp;
		}
	}

	// (2) save current images as history
	for(int i=0;i<img_count;i++)
		all_cameras_have_feature_points = all_cameras_have_feature_points*kpts_vec[i].size();
	if(all_cameras_have_feature_points != 0)
	{
		old_kpts_vec_stack.push(kpts_vec);
		old_imgs_pyr_stack.push(imgs_pyr);
		old_extrinsics_stack.push(extrinsics);
		count = count+1;
	}

	if(count >= TRACEBACK_MEMORY_LENGTH)
		MEMORY_RELEASE_FLAG = false;
	else
		MEMORY_RELEASE_FLAG = true;

	return kpts_intra_cam;
}



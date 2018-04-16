#include "myproject/MyReconstr_Vision.h"

MyReconstr_Vision::MyReconstr_Vision()
{
	
}


MyReconstr_Vision::~MyReconstr_Vision()
{
	
}

vector<KeyPoint>  MyReconstr_Vision::feature_extraction(cv::Mat img)
{
	vector<KeyPoint> kpts;
	Ptr<FastFeatureDetector> detector=FastFeatureDetector::create();
	vector<Mat> descriptor;

	detector->detect(img,kpts,Mat());
	return kpts;
}


vector<vector<KeyPoint> >  MyReconstr_Vision::feature_extraction_for_images(vector<cv::Mat> img_vec)
{
	int img_cnt = img_vec.size();
	vector<vector<KeyPoint> >  kpts_vec;

	for(int i=0; i<img_cnt; i++)
	{
		vector<KeyPoint> row = feature_extraction(img_vec[i]);
		kpts_vec.push_back(row);
	}

	return kpts_vec;
}


cv::Mat MyReconstr_Vision::draw_feature_points(cv::Mat src,vector<KeyPoint> kpts)
{
	cv::Mat img;
	drawKeypoints(src, kpts, img);
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


cv::Mat MyReconstr_Vision::image_collage_maker(vector<cv::Mat> src_vec)
{
	int img_cnt = src_vec.size();
	int rows = floor(sqrt(img_cnt));
	int cols = rows;
	vector<cv::Mat> img_vec(cols*rows);
	cv::Mat img;

	for(int i=1; i<img_cnt; i++)
	{
		if(src_vec[i].rows != src_vec[0].rows || src_vec[i].cols != src_vec[0].cols)
		{
			CONSOLE.visual_processing_error(1);
			exit(1);
		}
	}

	while(cols*rows<img_cnt)
		cols = cols+1;

	for(int i=0; i<cols*rows; i++)
	{
		if(i<img_cnt)
			resize(src_vec[i], img_vec[i], cv::Size(), double(1.0/cols), double(1.0/cols));
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


cv::Mat MyReconstr_Vision::process_image2D(vector<cv::Mat> curr_imgs)
{
	cv::Mat image2D;
	vector<vector<KeyPoint> >  kpts_vec = feature_extraction_for_images(curr_imgs);
	vector<cv::Mat> img_vec = draw_feature_points_for_images(curr_imgs,kpts_vec);
	image2D = image_collage_maker(img_vec);

	// ToDo: Replace the following baby script with real stuff!
	/* 
	int r=10; 
	int c=24;
	int b=1;  // B,G,R
	Images2D.at<cv::Vec3b>(r,c)[b] = 27; */
	return image2D;
}


PointCloud<PointXYZRGB> MyReconstr_Vision::process_model3D()
{
	PointCloud<PointXYZRGB> model3D;
	// ToDo: Replace the following baby script with real stuff!
	// ref: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html

	model3D.clear(); // Removes all points in a cloud and sets the width and height to 0. 
	PointXYZRGB pt1, pt2;
	pt1.x = 1;	pt1.y = 2;	pt1.z = 50;	pt1.r = 100;	pt1.g = 100;	pt1.b = 125;
	pt2.x = 1;	pt2.y = 57;	pt2.z = 5;  	pt2.r = 223;	pt2.g = 12;	pt2.b = 50;
	model3D.points.push_back(pt1);
	model3D.points.push_back(pt2);
	// to access each point: Model3D.points[0], Model3D.points[1]

	return model3D;
}



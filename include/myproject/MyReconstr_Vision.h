#ifndef MYRECONSTR_VISION_H
#define MYRECONSTR_VISION_H

#include "myproject/MyReconstr_Storage.h"
class MyReconstr_Vision
{
	private:
		MyReconstr_Display CONSOLE;
		FEATURE_ALGO_LIST MY_FEATURE_ALGO; 

	public:
		MyReconstr_Vision();
		~MyReconstr_Vision();	

		void set_feature_detection_algo(FEATURE_ALGO_LIST);
		void display_all_feature_algos(cv::Mat);

		vector<KeyPoint>  feature_extraction(cv::Mat);
		vector<KeyPoint>  feature_extraction(cv::Mat,FEATURE_ALGO_LIST);
		vector<vector<KeyPoint> >  feature_extraction_for_images(vector<cv::Mat>);

		cv::Mat draw_feature_points(cv::Mat,vector<KeyPoint>);
		vector<cv::Mat> draw_feature_points_for_images(vector<cv::Mat>,vector<vector<KeyPoint> >);
		cv::Mat image_collage_maker(vector<cv::Mat>);

		cv::Mat process_image2D(vector<cv::Mat>);
		PointCloud<PointXYZRGB> process_model3D();
};

#endif

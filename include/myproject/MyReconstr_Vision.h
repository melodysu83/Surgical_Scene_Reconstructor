#ifndef MYRECONSTR_VISION_H
#define MYRECONSTR_VISION_H

#include "myproject/MyReconstr_Imagefunc.h"
class MyReconstr_Vision
{
	private:
		MyReconstr_Display CONSOLE;
		FEATURE_ALGO_LIST MY_FEATURE_ALGO; 

		Ptr<FeatureDetector> detector_fast;
		Ptr<FeatureDetector> detector_surf;
		Ptr<FeatureDetector> detector_orb;

		// for FAST_ALGO
		int FEATURE_PARAM_INTENSITY_THRES;
		bool FEATURE_PARAM_NON_MAX_SUPPRE;

		// for SURF_ALGO
		double FEATURE_PARAM_HESSIAN_THRES;
		int FEATURE_PARAM_N_PYRAMIDS;
		int FEATURE_PARAM_N_PYRAMID_LAYERS;
		bool FEATURE_PARAM_DESCRIPTOR_EXTENDED;

		// for ORB_ALGO
		int FEATURE_PARAM_N_FEATURES;
		float FEATURE_PARAM_SCALE;
		int FEATURE_PARAM_N_LEVELS;
		int FEATURE_PARAM_EDGE_THRES;

		MyReconstr_Imagefunc IMGFUNC;

	public:
		MyReconstr_Vision();
		~MyReconstr_Vision();	

		void reset_feature_parameters();
		void set_feature_parameters_fast(int,bool);
		void set_feature_parameters_surf(double,int,int,bool);
		void set_feature_parameters_orb(int,float,int,int);
		void show_feature_parameters(FEATURE_ALGO_LIST);
		void show_all_feature_parameters();
		void create_feature_detectors();

		void set_feature_detection_algo(FEATURE_ALGO_LIST);
		void display_all_feature_algos(cv::Mat);

		vector<KeyPoint>  feature_extraction(cv::Mat);
		vector<KeyPoint>  feature_extraction(cv::Mat,FEATURE_ALGO_LIST);
		vector<vector<KeyPoint> >  feature_extraction_for_images(vector<cv::Mat>);
		vector<vector<KeyPoint> >  feature_extraction_for_images(vector<cv::Mat>,FEATURE_ALGO_LIST);

		cv::Mat draw_feature_points(cv::Mat,vector<KeyPoint>);
		vector<cv::Mat> draw_feature_points_for_images(vector<cv::Mat>,vector<vector<KeyPoint> >);
		cv::Mat image_collage_maker(vector<cv::Mat>);

		cv::Mat process_image2D(vector<cv::Mat>);
		PointCloud<PointXYZRGB> process_model3D();
};

#endif

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

		vector<vector<Point2f> >  features_intra_cam;
		vector<vector<Point2f> >  features_inter_cam;

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

		// parameters for reconstruction
		int pyramid_maxlayer;
		int traceback_length;
		int pyramid_winsize;
		cv::Size pyramid_window;

		vector<cv::Mat> CALI_INTRI_DATA;
		vector<cv::Mat> CALI_DISTO_DATA;
		bool MEMORY_INSUFFICIENT_FLAG;
		bool MEMORY_DEEPCLEAR_FLAG;

		MyReconstr_Imagefunc IMGFUNC;
		pthread_mutex_t param_update_mutex;

	public:
		MyReconstr_Vision();
		~MyReconstr_Vision();	

		void load_intrinsic_matrices(vector<cv::Mat>);
		void load_distortion_matrices(vector<cv::Mat>);
		void reset_feature_parameters();
		void reset_reconstr_parameters();
		void reset_processing();
		void set_feature_parameters_fast(int,bool);
		void set_feature_parameters_surf(double,int,int,bool);
		void set_feature_parameters_orb(int,float,int,int);
		void set_reconstr_parameters(int,int,int);
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
		cv::Mat draw_feature_intra_cam_matches(cv::Mat,vector<Point2f>,vector<Point2f>);
		cv::Mat draw_feature_inter_cam_matches(cv::Mat,cv::Mat,vector<Point2f>,vector<Point2f>);
		vector<cv::Mat> draw_feature_points_for_images(vector<cv::Mat>,vector<vector<KeyPoint> >);
		vector<cv::Mat> draw_feature_intra_cam_matches_for_images(vector<cv::Mat>,vector<vector<Point2f> > );
		vector<cv::Mat> draw_feature_inter_cam_matches_for_images(vector<cv::Mat>,vector<vector<Point2f> > ,vector<vector<int> >);
		cv::Mat image_collage_maker(vector<cv::Mat>,bool);

		cv::Mat process_image2D(vector<cv::Mat>,vector<vector<double> >,vector<int>);
		PointCloud<PointXYZRGB> process_model3D();

		vector<vector<Point2f> > feature_tracking_cross_camera(vector<vector<Point2f> >,vector<vector<cv::Mat> >,vector<cv::Mat>,vector<cv::Mat>,vector<cv::Mat>,vector<vector<int> >);
		vector<vector<Point2f> > feature_tracking_inter_camera(vector<vector<Point2f> >,vector<vector<cv::Mat> >,vector<cv::Mat>,vector<vector<int> >);
		vector<vector<Point2f> > feature_tracking_intra_camera(vector<vector<Point2f> >,vector<vector<cv::Mat> >,vector<cv::Mat>);
		
};

#endif

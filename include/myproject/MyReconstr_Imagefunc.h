#ifndef MYRECONSTR_IMAGEFUNC_H
#define MYRECONSTR_IMAGEFUNC_H

#include "myproject/MyReconstr_Map.h"

class MyReconstr_Imagefunc
{
	private:
		double sigma; 	// feature detection uncertainty
		double epsilon;	// some small number to avoid invalid denominator
		double pi;
		cv::Mat I2;
		cv::Mat I3;
		cv::Mat I4;
		MyReconstr_Display CONSOLE;
	public:
		MyReconstr_Imagefunc();
		~MyReconstr_Imagefunc();

		double get_sigma();		// get current sigma value
		void set_sigma(double);		// set new sigma value
		double norm_1(cv::Mat);		// compute the 1-norm of vector
		double norm_2(cv::Mat);		// compute the 2-norm of vector

		unsigned long int factorial(int);
		vector<vector<int> > constrained_combination(vector<int>,bool);

		// Common Abbreviations:
		// Wpt: world point in the real world 		(size: 3x1 or 4x1)
		// Mpt: map point in the reconstructed map 	(size: 3x1 or 4x1)
		// Ipt: image point from the camera views	(size: 2x1 or 3x1)
		// P:   the projection matrix                   (size: 2x3 or 3x4)
		// J:   the jacobian of the projection matrix P (size: 2x3)
		// Js:  the stack of J matrices from all views  (size: (2k)x3)
		// E:   the covariance matrix for map points    (size: 3x3)
		// c:   the covariance matrix for image points  (size: 2x2)
		// K:   the kalman gain for points uncertainty  (size: 3x2)

		bool check_if_normalized_Ipt(cv::Mat);
		bool check_if_normalized_Mpt(cv::Mat);
		bool check_if_normalized_P(cv::Mat);
		bool check_if_valid_Ipt(cv::Mat);
		bool check_if_valid_Mpt(cv::Mat);
		bool check_if_valid_P(cv::Mat);
		bool check_if_valid_J(cv::Mat);
		bool check_if_valid_Js(cv::Mat);
		bool check_if_valid_E(cv::Mat);
		bool check_if_valid_c(cv::Mat);
		bool check_if_valid_K(cv::Mat);
		bool check_if_valid(cv::Mat,int,int);

		cv::Mat project_Wpt_to_Ipt(cv::Mat,cv::Mat);			// projection operation
		vector<cv::Mat> project_Wpt_to_Ipt(cv::Mat,vector<cv::Mat>);	// projection operation (in bulks)
		cv::Mat compute_projection_jacobian_J(cv::Mat);			// the jacobian of projection function
		cv::Mat to_scalable_form(cv::Mat);				// [x,y,z] --> [x,y,z,1]
		cv::Mat to_normalized_form(cv::Mat);				// [x,y,z,1] --> [x,y,z]
		cv::Mat pose_to_transMat(vector<double>,bool);
		vector<double> transMat_to_pose(cv::Mat,bool);
		vector<cv::Mat> poses_to_transMats(vector<vector<double> >,bool);
		vector<vector<double> > transMats_to_poses(vector<cv::Mat>,bool);
		cv::Mat pose_to_projMat(vector<double>,bool);
		vector<double> projMat_to_pose(cv::Mat,bool);
		vector<cv::Mat> poses_to_projMats(vector<vector<double> >,bool);
		vector<vector<double> > projMats_to_poses(vector<cv::Mat>,bool);


		double tukey_biweight_function(double);					// eq. (2) 
		double tukey_biweight_function(double,double);
		double mahalanobis_distance(cv::Mat,cv::Mat,cv::Mat);			// eq. (8)
		double mahalanobis_distance(cv::Mat,cv::Mat,cv::Mat,cv::Mat);		// eq. (8)
		double evaluate_Ipt_to_Mpt_correspondance(cv::Mat,cv::Mat,cv::Mat);	// eq. (1) - first half

		cv::Mat compute_Mpt_uncertainty_covariance_E(cv::Mat);			// eq. (4)
		cv::Mat update_Mpt_from_Ipt(cv::Mat,cv::Mat,cv::Mat,cv::Mat);		// eq. (5)
		cv::Mat update_kalman_gain(cv::Mat,cv::Mat);				// eq. (6)
		cv::Mat update_Mpt_uncertainty_covariance_E(cv::Mat,cv::Mat,cv::Mat);	// eq. (7)	
};
#endif

#include "myproject/MyReconstr_Imagefunc.h"

MyReconstr_Imagefunc::MyReconstr_Imagefunc()
{
	this->sigma = DEFAULT_IMAGE_FUNC_SIGMA;
	this->I2 = cv::Mat::eye(2,2, CV_32F);
	this->I3 = cv::Mat::eye(3,3, CV_32F);
}


MyReconstr_Imagefunc::~MyReconstr_Imagefunc()
{
}


double MyReconstr_Imagefunc::get_sigma() // get current sigma value
{
	double result = this->sigma;	
	return result;
}


void MyReconstr_Imagefunc::set_sigma(double sig)// set new sigma value
{
	this->sigma = sig; //ToDo: actually call this function somewhere!~
}


double MyReconstr_Imagefunc::norm_1(cv::Mat pt)	// compute the 1-norm of vector
{
	if(pt.cols != 1)
	{
		CONSOLE.image_tool_function_error(0);
		return -1;
	}
	
	double result = 0;
	for(int i=0; i<pt.rows; i++)
		result = result + abs(pt.at<float>(i,0));

	return result; 
}


double MyReconstr_Imagefunc::norm_2(cv::Mat pt)	// compute the 2-norm of vector
{
	if(pt.cols != 1)
	{
		CONSOLE.image_tool_function_error(1);
		return -1;
	}

	double result = 0;
	for(int i=0; i<pt.rows; i++)
		result = result + pt.at<float>(i,0)*pt.at<float>(i,0);

	return result; 
}


cv::Mat MyReconstr_Imagefunc::project_Wpt_to_Ipt(cv::Mat proj_mat,cv::Mat pt)	// projection operation
{
	if(pt.cols != 1 || (pt.rows != 3 && pt.rows != 4) || (proj_mat.rows != 2 && proj_mat.rows != 3))
	{
		CONSOLE.image_tool_function_error(2);
		return cv::Mat();
	}
	if(pt.rows == 3 && proj_mat.cols == 4)
	{
		cv::Mat m = cv::Mat::ones(4, 3, CV_64F);    // 3 cols, 4 rows
		cv::Mat row = cv::Mat::ones(1, 3, CV_64F);  // 3 cols
		m.push_back(row);    
	}
	else if(pt.rows == 4 && proj_mat.cols == 3)
	{
		
	}

	cv::Mat tmp, result;
	tmp = proj_mat * pt;

	if(tmp.rows == 4)
	{
		result = tmp(Range(0,tmp.rows-1), Range(0, tmp.cols)).clone();
		result = result / tmp.at<float>(tmp.rows-1,0);
	}
	else
		result = tmp;

	return result.clone();
}


vector<cv::Mat> MyReconstr_Imagefunc::project_Wpt_to_Ipt(cv::Mat proj_mat,vector<cv::Mat> pts) // projection operation (in bulks)
{
	vector<cv::Mat> result(pts.size());
	for(int i=0; i<pts.size(); i++)
	{
		result[i] = project_Wpt_to_Ipt(proj_mat,pts[i]);
	}

	return result;
}


cv::Mat MyReconstr_Imagefunc::compute_projection_jacobian_J(cv::Mat proj_mat)	// the jacobian of projection function
{
	if( (proj_mat.cols != 2 && proj_mat.cols != 3) || (proj_mat.rows != 4 && proj_mat.rows != 3))
	{
		CONSOLE.image_tool_function_error(3);
		return cv::Mat();
	}

	cv::Mat result = proj_mat(Range(0,2), Range(0, 3));
	return result.clone();
}


cv::Mat MyReconstr_Imagefunc::to_scalable_form(cv::Mat m)
{
	cv::Mat result = m.clone();
	cv::Mat row = cv::Mat::zeros(1,m.cols,m.type());
	row.at<float>(0,m.cols-1) = 1;

	result.push_back(row);
	if(result.rows != m.rows+1 || result.cols != m.cols)
		CONSOLE.image_tool_function_error(5);

	return result;
}


cv::Mat MyReconstr_Imagefunc::to_normalized_form(cv::Mat m)
{
	cv::Mat result(m.rows-1,m.cols,m.type());
	if(m.cols == 1)
	{
		result = m(Range(0,m.rows-1),Range(0,m.cols));
		if(m.at<float>(m.rows-1,0) != 0)
			result = result / (m.at<float>(m.rows-1,0));
		else
			CONSOLE.image_tool_function_error(6);
	}
	else
	{
		result = m(Range(0,m.rows-1),Range(0,m.cols));
	}
	
	return result;
}


bool MyReconstr_Imagefunc::check_if_normalized_Ipt(cv::Mat Ipt)
{
	if(check_if_valid_Ipt(Ipt))
		return Ipt.rows == 2;
	else
	{
		CONSOLE.image_tool_function_error(10);
		return false;
	}
}


bool MyReconstr_Imagefunc::check_if_normalized_Mpt(cv::Mat Mpt)
{
	if(check_if_valid_Mpt(Mpt))
		return Mpt.rows == 3;
	else
	{
		CONSOLE.image_tool_function_error(11);
		return false;
	}
}


bool MyReconstr_Imagefunc::check_if_normalized_P(cv::Mat P)
{
	if(check_if_valid_P(P))
		return P.rows == 2;
	else
	{
		CONSOLE.image_tool_function_error(12);
		return false;
	}
}


bool MyReconstr_Imagefunc::check_if_valid_Ipt(cv::Mat Ipt)
{
	bool result = check_if_valid(Ipt,3,1) || check_if_valid(Ipt,2,1);

	if(!result)
		CONSOLE.image_tool_function_error(7);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_Mpt(cv::Mat Mpt)
{
	bool result = check_if_valid(Mpt,4,1) || check_if_valid(Mpt,3,1);

	if(!result)
		CONSOLE.image_tool_function_error(8);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_P(cv::Mat P)
{
	bool result = check_if_valid(P,3,4) || check_if_valid(P,2,3);

	if(!result)
		CONSOLE.image_tool_function_error(9);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_J(cv::Mat J)
{
	bool result = check_if_valid(J,2,3);

	if(!result)
		CONSOLE.image_tool_function_error(13);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_Js(cv::Mat Js)
{
	bool result = (Js.rows == 3) && ((Js.cols%2) == 0);

	if(!result)
		CONSOLE.image_tool_function_error(28);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_E(cv::Mat E)
{
	bool result = check_if_valid(E,3,3);

	if(!result)
		CONSOLE.image_tool_function_error(15);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_c(cv::Mat c)
{
	bool result = check_if_valid(c,2,2);

	if(!result)
		CONSOLE.image_tool_function_error(16);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid_K(cv::Mat K)
{
	bool result = check_if_valid(K,3,2);

	if(!result)
		CONSOLE.image_tool_function_error(22);

	return result;
}


bool MyReconstr_Imagefunc::check_if_valid(cv::Mat M,int rows,int cols)
{
	return M.rows == rows && M.cols == cols;
}


double MyReconstr_Imagefunc::tukey_biweight_function(double x)	// eq. (2)
{
	double t = 3*this->sigma;
	return tukey_biweight_function(x,t);
}


double MyReconstr_Imagefunc::tukey_biweight_function(double x, double t) // eq. (2)
{
	if(abs(x)<=t)
		return pow(t,2)*(1-pow(1-pow(1.0*x/t,2),3))/6.0;
	else
		return pow(t,2)/6.0;
}


double MyReconstr_Imagefunc::mahalanobis_distance(cv::Mat pti,cv::Mat ptj,cv::Mat ci) // eq. (8)
{
	if(!check_if_valid_Ipt(pti) ||!check_if_valid_Ipt(ptj) ||!check_if_valid_c(ci) )
	{
		CONSOLE.image_tool_function_error(18);
		return -1;
	}
	cv::Mat tmp = pti-ptj;
	cv::Mat result = tmp.t()*ci.inv()*tmp;
	return result.at<float>(0,0);
}


double MyReconstr_Imagefunc::mahalanobis_distance(cv::Mat pti,cv::Mat ptj,cv::Mat Ei,cv::Mat Ji)
{
	if(!check_if_valid_Ipt(pti) ||!check_if_valid_Ipt(ptj) ||!check_if_valid_E(Ei) ||!check_if_valid_J(Ji) )
	{
		CONSOLE.image_tool_function_error(17);
		return -1;
	}
	cv::Mat ci = Ji*Ei*Ji.t() + this->sigma*(I2);

	return mahalanobis_distance(pti,ptj,ci);
}


double MyReconstr_Imagefunc::evaluate_Ipt_to_Mpt_correspondance(cv::Mat P, cv::Mat Mpt,cv::Mat Ipt) // eq. (1) - first half
{
	if(!check_if_valid_Ipt(Ipt) || !check_if_valid_Mpt(Mpt) || !check_if_valid_P(P) )
	{
		CONSOLE.image_tool_function_error(4);
		return -1;
	}

	if(check_if_normalized_Mpt(Mpt))
		Mpt = to_scalable_form(Mpt);

	if(check_if_normalized_Ipt(Ipt))
		Ipt = to_scalable_form(Ipt);

	if(check_if_normalized_P(P))
		P = to_scalable_form(P);

	cv::Mat dist_vec = to_normalized_form(Ipt - project_Wpt_to_Ipt(P,Mpt));
	double dist = norm_1(dist_vec);

	return tukey_biweight_function(dist);
}


cv::Mat MyReconstr_Imagefunc::compute_Mpt_uncertainty_covariance_E(cv::Mat Js) // eq. (4)
{
	cv::Mat E;
	if(!check_if_valid_Js(Js))
		CONSOLE.image_tool_function_error(14);
	else
		E = pow(sigma,2)*(Js.t()*Js).inv();
	
	return E.clone();	
}


cv::Mat MyReconstr_Imagefunc::update_Mpt_from_Ipt(cv::Mat Mpt,cv::Mat Ipt,cv::Mat P,cv::Mat K) // eq. (5)
{
	cv::Mat Mpt_new = cv::Mat();

	if(!check_if_valid_K(K))
		CONSOLE.image_tool_function_error(19);

	else if(!check_if_valid_P(P))
		CONSOLE.image_tool_function_error(20);

	else if(!check_if_valid_Mpt(Mpt) || !check_if_valid_Ipt(Ipt) )
		CONSOLE.image_tool_function_error(21);

	else
	{
		if(check_if_normalized_Mpt(Mpt))
			Mpt = to_scalable_form(Mpt);

		if(check_if_normalized_Ipt(Ipt))
			Ipt = to_scalable_form(Ipt);

		if(check_if_normalized_P(P))
			P = to_scalable_form(P);
		
		Mpt_new = to_normalized_form(Mpt + K*(project_Wpt_to_Ipt(P,Mpt)-Ipt));
		
		if(!check_if_valid_Mpt(Mpt_new))
		{
			CONSOLE.image_tool_function_error(23);
			Mpt_new = cv::Mat();
		}
	}
	return Mpt_new;
}


cv::Mat MyReconstr_Imagefunc::update_kalman_gain(cv::Mat J, cv::Mat E) // eq. (6)
{
	cv::Mat K = cv::Mat();

	if(!check_if_valid_J(J) || !check_if_valid_E(E))
		CONSOLE.image_tool_function_error(25);

	K = E*J.t()*((sigma*I3+J*E*J.t()).inv());

	if(!check_if_valid_K(K))
	{
		CONSOLE.image_tool_function_error(24);
		K = cv::Mat();
	}
	
	return K;
}


cv::Mat MyReconstr_Imagefunc::update_Mpt_uncertainty_covariance_E(cv::Mat E, cv::Mat K, cv::Mat J) // eq. (7)	
{
	if(!check_if_valid_E(E) || !check_if_valid_K(K) || !check_if_valid_J(J))
		CONSOLE.image_tool_function_error(26);

	cv::Mat E_new = cv::Mat();
	E_new = E - K*J*E;

	if(!check_if_valid_E(E_new))
	{
		CONSOLE.image_tool_function_error(27);
		E_new = cv::Mat();
	}
	return E_new;
}




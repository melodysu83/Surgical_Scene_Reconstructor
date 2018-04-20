#include "myproject/MyReconstr_Dependencies.h"
string FEATURE_ALGO_TO_STRING(FEATURE_ALGO_LIST feat_algo)
{
	switch(feat_algo)
	{
         	case FAST_ALGO:
            		return "FAST_ALGO";
         	case SURF_ALGO:
            		return "SURF_ALGO";
		case ORB_ALGO:
			return "ORB_ALGO";
         	default:
            		return "INVALID ENUM";
	}
 }

#ifndef MYRECONSTR_MAP_H
#define MYRECONSTR_MAP_H

#include "myproject/MyReconstr_Points.h"

/*
  Design Idea for MyReconstr_Map:
	1. ( ) The map object contains several map_pts
	2. ( ) Upgrades homeless_pts to map_points if they last long enough
	3. ( ) Remove map_pt from list if childless for too long

  Considering...
	1. ( ) Hash table for easier point finding? Best data structure for this implementation
	2. ( ) ...
	
*/

class MyReconstr_Map
{
	private:
		int camera_cnt;
		vector<MyReconstr_Point3D> map_pt;
		vector<vector<MyReconstr_Point2D> > homeless_pts;
	public:
		MyReconstr_Map(int);
		~MyReconstr_Map();
		
};
#endif

#ifndef MYRECONSTR_POINTS_H
#define MYRECONSTR_POINTS_H

#include "myproject/MyReconstr_Storage.h"

class MyReconstr_Point3D; // forward declaration


/*
  Design Idea for MyReconstr_Point2D:
	1. ( ) Every point2D has a 2D position
	2. ( ) Pointers to corresponding 3D point "map_pt", i.e. it's parent
	   ( ) And it points to the empty 3D point if there is no existing 3D map points that it corresponds to (set homeless flag)
	3. ( ) Counting variable to keep track of how long a point2D is homeless
	4. ( ) If the count is greater than threshold: Create new map point for it:
	       Create new Point3D for it if generation is big
	       Remove it entirely if generation is small.
	5. ( ) Iteratively update the pointers to Point3D parent (set homeless flag is needed)
	6. ( ) History tracking: update generation value.

  ToDo:
	1. ( ) Create empty 2D point ()
	2. ( ) develope policy to: Generate a new Point2D object (merge the matching results? use descriptors instead of points?)
	   ( )                     Update location of a Point2D object
	3. ( ) Refresh Point2D: move pt->pt_old list, and pop one from old list?

  Considering...
	1. ( ) Add connection to matched points too? --> when it's homeless, it'll be helpful... or create another homeless_points class?
	
*/

class MyReconstr_Point2D
{
	private:
		bool homeless;     // no map_pt correspondance
		int memory_size;   // max generation stored
		int generation;	   // how many old points has been tracked through time
		int homeless_cnt;  // how long has the point been homeless
		Point2f pt;
		cv::Mat campose;
		queue<Point2f> old_pt;
		queue<cv::Mat> old_campose;
		MyReconstr_Point3D* map_pt;
	public:
		MyReconstr_Point2D(int); // Memory length ToDo: update "Memory length" accordingly with Vision class
					  // ToDo: set points pt
				          // ToDo: set old_points... maybe not
		~MyReconstr_Point2D(); 
};


/*
  Design Idea for MyReconstr_Point3D:
	1. ( ) Every point3D has a 3D position
	2. ( ) Pointers to corresponding 2D points "img_pt" from all cameras
	   ( ) And they point to the empty 2D point if there is no correspondance to image from that particular camera
	3. ( ) Counting variable to keep track of how long a point3D is childless
	4. ( ) Iteratively update the pointers to Point2D children (set childless flag is needed)

  ToDo:
	1. ( ) compute uncertainty as well...? (related to section 5.1 in COSLAM paper)
	2. ( ) Create empty 2D point ()
	3. ( ) develope policy to: Generate a new Point3D object (maybe in Map class)
	   ( )                 Update location of a Point3D object (maybe in Map class)
	   ( )                 Remove a Point3D object: childless for too long (maybe in Map class)

  Considering...
	1. ( ) point classification: create an enum for it instead of int type?

*/
class MyReconstr_Point3D
{
	private:
		bool childless;    
		vector<bool> visible;     // whether it's visible from that particular camera a vector of length == camera_count
		int camera_cnt;    // the number of cameras
		int childless_cnt; // how long has the point been childless
		int type;	   // static? moving? deforming? 
		Point3f pt;
		vector<MyReconstr_Point2D*> img_pt;  // img_pt[size = camera count], each of them is an address for the point2D
	public:
		MyReconstr_Point3D(int); // camera count
		~MyReconstr_Point3D(); 
		
};
#endif

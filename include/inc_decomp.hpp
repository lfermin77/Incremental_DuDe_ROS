//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//DuDe
#include "wrapper.hpp"



class Stable_graph
{
	public:
	
//	Nodes
	std::vector<std::vector<cv::Point> > Region_contour;
	std::vector<cv::Point> Region_centroid;
	std::vector<std::set<int> > Region_connections;

// Edges
	std::vector<cv::Point> diagonal_centroid;
	std::vector<std::set<int> > diagonal_connections;
	
	cv::Size image_size;
	
	
	cv::Mat draw_stable_contour();
	
};





class Incremental_Decomposer{
	public:
	cv::Point2f current_origin_;
	cv::Point2f new_origin_;
	float resolution;
	Stable_graph Stable;
	
	float Decomp_threshold_;
	float safety_distance;
	
	bool first_time;
	cv::Rect previous_rect;
	
	
	Incremental_Decomposer();
	
	~Incremental_Decomposer();


/////////////////////////////////
///// MAIN FUNCTIONS	
	Stable_graph decompose_image(cv::Mat image_cleaned, float pixel_Tau, cv::Point2f origin, float resolution_in);
	
/////////////////////////////
//// UTILITIES
		void are_contours_connected(vector<cv::Point> first_contour, vector<cv::Point> second_contour, cv::Point &centroid, int &number_of_ones );
		
		void adjust_stable_contours();
		
		cv::Point pixel_to_cartesian(cv::Point point_in);
		
		cv::Point cartesian_to_pixel(cv::Point point_in);		
};


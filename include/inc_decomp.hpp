//openCV
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
	
// Frontiers
	std::vector<cv::Point> center_of_frontier;
	std::vector<int > region_frontier;	
	
	cv::Size image_size;
	std::map<int, int > index_to_color;
	int max_color;


	
	Stable_graph();
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
		
		void frontiers_in_map(cv::Mat  Tag_image, cv::Mat  original_image);
		
		cv::Point contour_centroid(std::map < std::set<int> , std::vector<cv::Point>   >  input_map);
		std::vector < std::vector<cv::Point> > decompose_edge(	 std::vector<cv::Point>   points_in_edge);		
};


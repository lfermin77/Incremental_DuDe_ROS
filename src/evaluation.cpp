//ROS
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

//DuDe
#include "inc_decomp.hpp"





class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_, it2_;
	image_transport::Subscriber image_sub_, image_sub2_;
	image_transport::Publisher image_pub_, image_pub2_;	
	cv_bridge::CvImagePtr cv_ptr, cv_ptr2;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Timer timer;
			
	float Decomp_threshold_;
	bool segmentation_ready;
	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;

	
	public:
		ROS_handler(const std::string& mapname, float threshold) : mapname_(mapname),  it_(n), it2_(n), Decomp_threshold_(threshold)
		{
			ROS_INFO("Waiting for the map");
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			segmentation_ready = false;
			
			image_pub_  = it_.advertise("/ground_truth_segmentation", 1);			
			image_pub2_ = it_.advertise("/DuDe_segmentation",   1);			

			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";

			cv_ptr2.reset (new cv_bridge::CvImage);
			cv_ptr2->encoding = "mono8";
			
			read_files();
						
		}


/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  if(segmentation_ready) publish_Image();
		}




////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish (cv_ptr->toImageMsg());
			image_pub2_.publish(cv_ptr2->toImageMsg());
		}



/////////////////////////
//// UTILITY
/////////////////////////

		typedef std::map <std::vector<int>, std::vector <cv::Point> > match2points;
		typedef std::map <int, std::vector <cv::Point> > tag2points;
	//////////////////////
		void read_files(){
			cv::Mat image_GT, image_original;
//			image_original    = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan.png",0);   // Read the file
			image_original    = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan_furnitures_trashbins.png",0);   // Read the file
			image_GT          = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan_gt_segmentation.png",0);   // Read the file

			
			cv::Mat GT_segmentation = simple_segment(image_GT);

			double begin_process, end_process, decompose_time;
			begin_process = getTime();
			
			cv::Mat DuDe_segmentation = simple_segment(image_original);

			end_process = getTime();	decompose_time = end_process - begin_process;			
			std::cout << "Time to decompose " << decompose_time << std::endl;
			
			
			
			
			match2points relations2points = compare_images(GT_segmentation, DuDe_segmentation);
						
			/////////////
			cv::Mat to_publish = GT_segmentation;
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish.convertTo(to_publish, CV_32F);
			to_publish.copyTo(cv_ptr->image);////most important
			////////////
			cv::Mat to_publish2 = DuDe_segmentation;
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish2, CV_32F);
			to_publish2.copyTo(cv_ptr2->image);////most important
			///////////
			
			segmentation_ready = true;
		}

	/////////////////////
		match2points compare_images(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in){
			match2points relation2points;
			tag2points GT_tag2points, DuDe_tag2points;
			
			cv::Mat GT_segmentation   = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
			cv::Mat DuDe_segmentation = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
			
			GT_segmentation_in  .convertTo(GT_segmentation, CV_8UC1);
			DuDe_segmentation_in.convertTo(DuDe_segmentation, CV_8UC1);			
			
			for(int x=0; x < GT_segmentation.size().width; x++){
				for(int y=0; y < GT_segmentation.size().height; y++){
					cv::Point current_pixel(x,y);
					std::vector < int > relation;
										
					int tag_GT   = GT_segmentation.at<uchar>(current_pixel);
					int tag_DuDe  = DuDe_segmentation.at<uchar>(current_pixel);
					
					if(tag_DuDe>0){
						relation.push_back( tag_GT );
						relation.push_back( tag_DuDe );
	
						relation2points[relation].push_back(current_pixel);					
						GT_tag2points  [tag_GT].push_back(current_pixel);
						DuDe_tag2points[tag_DuDe].push_back(current_pixel);
					}
				}
			}
			


			for( match2points::iterator it = relation2points.begin(); it!= relation2points.end(); it++ ){
				std::vector < int > current_relation = it->first;
				std::vector < cv::Point > current_points = it->second;
				
				if(current_points.size() > (GT_segmentation.rows * GT_segmentation.rows)/100){
					int points_in_GT, points_in_DuDe, points_in_both;
					
					points_in_GT = GT_tag2points[current_relation[0]].size();
					points_in_DuDe = DuDe_tag2points[current_relation[1]].size();
					points_in_both = current_points.size();
					
					std::cout << "Relation ("<< current_relation[0] << "," << current_relation[1] << ") with " << current_points.size() << " points" << std::endl;
					std::cout << "   Precision: "<<  100*points_in_both/points_in_GT << "%, Recall: " << 100*points_in_both/points_in_DuDe<<"% " << std::endl;
				}
			}
			 /*
			for( tag2points::iterator it = GT_tag2points.begin(); it!= GT_tag2points.end(); it++ ){
				std::cout << "GT Tags "<< *it << std::endl;
			}
			for( std::set <int>::iterator it = DuDe_tags.begin(); it!= DuDe_tags.end(); it++ ){
				std::cout << "DuDe tags "<< *it << std::endl;
			}
			//*/
			std::cout << "relation2points size "<< relation2points.size() << std::endl;
			return relation2points;
		}

	///////////////////
		void read_file(){
			cv::Mat image_GT, image_original;
			image_original    = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan.png",0);   // Read the file
			image_GT          = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan_gt_segmentation.png",0);   // Read the file
			
			if(! image_original.data )                              // Check for invalid input
		    {
		        cout <<  "Could not open or find the image" << std::endl ;
		    }
			
			std::cout << "Original:     (" <<  image_original.rows <<" , "<<image_original.cols<<")" << std::endl;
			std::cout << "Ground Truth: (" <<  image_GT.rows <<" , "<<image_GT.cols<<")" << std::endl;

			cv::Point2f origin(0,0);

		/////////////////////////////////////// Decompose GT
			Incremental_Decomposer inc_decomp;
			Stable_graph Stable;
			
			cv::Mat pre_decompose = image_GT.clone();
			cv::Mat pre_decompose_BW = pre_decompose > 250;

			Stable = inc_decomp.decompose_image(pre_decompose_BW, 3.5, origin , 0.05);

			cv::Mat GT_segmentation = Stable.draw_stable_contour().clone();	
		//////////////////////////////////////// Decompose Original
			Incremental_Decomposer inc_decomp2;
			Stable_graph Stable2;

			cv::Mat pre_decompose2 = image_original.clone() ;			
			cv::Mat pre_decompose2_BW = pre_decompose2 > 250;			

			Stable2 = inc_decomp2.decompose_image(pre_decompose2_BW, 3.5, origin , 0.05);

			cv::Mat DuDe_segmentation = Stable2.draw_stable_contour().clone();	
			
			std::cout << "Decomposition DuDe     has "<< Stable2.Region_contour.size() << " tags"<<std::endl;
			std::cout << "Decomposition Original has "<< Stable .Region_contour.size() << " tags"<<std::endl;
		/////////////////////////////////////

//*

			match2points relation2points;
			std::set <int> GT_tags, DuDe_tags;
			for(int x=0; x < image_original.rows; x++){
				for(int y=0; y < image_original.cols; y++){
					
					cv::Point current_pixel(x,y);
					std::vector < int > relation;
					
					int tag_GT   = GT_segmentation.  at<uchar>(current_pixel);
					int DuDe_GT  = DuDe_segmentation.at<uchar>(current_pixel);
					
					GT_tags.insert(tag_GT); 
//					DuDe_tags.insert(DuDe_GT);
					
					relation.push_back( tag_GT );
					relation.push_back( DuDe_GT );

//					relation2points[relation].push_back(current_pixel);
					
//					std::cout << "GT Tags "<< GT_tag << std::endl;
					
				}
			}
			std::cout << "GT Tags size "  << GT_tags.size()   << std::endl;
			std::cout << "DuDe_tags size "<< DuDe_tags.size() << std::endl;
			
//*			
			for( match2points::iterator it = relation2points.begin(); it!= relation2points.end(); it++ ){
				std::vector < int > current_relation = it->first;
				std::vector < cv::Point > current_points = it->second;
//				std::cout << "Relation ("<< current_relation[0] << "," << current_relation[1] << ") with " << current_points.size() << " points" << std::endl;
			}

			for( std::set <int>::iterator it = GT_tags.begin(); it!= GT_tags.end(); it++ ){
				std::cout << "GT Tags "<< *it << std::endl;
			}
			for( std::set <int>::iterator it = DuDe_tags.begin(); it!= DuDe_tags.end(); it++ ){
				std::cout << "DuDe tags "<< *it << std::endl;
			}
//*/

		}
	////////////////////
		cv::Mat simple_segment(cv::Mat image_in){
			Incremental_Decomposer inc_decomp;
			Stable_graph Stable;
			cv::Point2f origin(0,0);
			float resolution = 0.05;

			
			cv::Mat pre_decompose = image_in.clone();
			cv::Mat pre_decompose_BW = pre_decompose > 250;

			Stable = inc_decomp.decompose_image(pre_decompose_BW, Decomp_threshold_/resolution, origin , resolution);

//			cv::Mat Segmentation = Stable.draw_stable_contour();
			
			cv::Mat Drawing = cv::Mat::zeros(image_in.size(), CV_8UC1);	
			for(int i = 0; i < Stable.Region_contour.size();i++){
				drawContours(Drawing, Stable.Region_contour, i, i+1, -1, 8);
			}
			std::cout << "Decomposition size: " << Stable.Region_contour.size() << std::endl;

			return Drawing;
		}
	
	////////////////////

};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "evaluation");
	
	std::string mapname = "map";
	
	float decomp_th=3;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(mapname, decomp_th);
	ros::spin();
	
	return 0;
}

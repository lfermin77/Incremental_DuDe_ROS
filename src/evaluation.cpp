//ROS
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

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
	Incremental_Decomposer inc_decomp;
	Stable_graph Stable;

	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;

	
	public:
		ROS_handler(const std::string& mapname, float threshold) : mapname_(mapname),  it_(n), it2_(n), Decomp_threshold_(threshold)
		{
			ROS_INFO("Waiting for the map");
//			map_sub_ = n.subscribe("map", 2, &ROS_handler::mapCallback, this); //mapname_ to include different name
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);

			image_pub_  = it_.advertise("/ground_truth_segmentation", 1);			
			image_pub2_ = it_.advertise("/DuDe_segmentation",   1);			

			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";

			cv_ptr2.reset (new cv_bridge::CvImage);
			cv_ptr2->encoding = "mono8";
			
			read_file();
						
		}


/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		

		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
		{
			double begin_process, end_process, begin_whole, occupancy_time, decompose_time, drawPublish_time, whole_time;
			begin_whole = begin_process = getTime();
			
			ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

		///////////////////////Occupancy to clean image	
			cv::Mat grad, img(map->info.height, map->info.width, CV_8U);
			img.data = (unsigned char *)(&(map->data[0]) );
			
			float pixel_Tau = Decomp_threshold_ / map->info.resolution;				
			cv_ptr->header = map->header;
			cv::Point2f origin = cv::Point2f(map->info.origin.position.x, map->info.origin.position.y);

			
			cv::Rect first_rect = find_image_bounding_Rect(img); 
			float rect_area = (first_rect.height)*(first_rect.width);
			float img_area = (img.rows) * (img.cols);
			cout <<"Area Ratio " <<  ( rect_area/img_area  )*100 <<"% "<< endl;
			
			cv::Mat cropped_img;
			img(first_rect).copyTo(cropped_img); /////////// Cut the relevant image

			cv::Mat black_image2, image_cleaned2 = clean_image2(cropped_img, black_image2);
			
			cv::Mat image_cleaned = cv::Mat::zeros(img.size(), CV_8UC1);
			cv::Mat black_image   = cv::Mat::zeros(img.size(), CV_8UC1);
			
			image_cleaned2.copyTo(image_cleaned (first_rect));
			black_image2.copyTo(black_image (first_rect));
			
			end_process = getTime();	occupancy_time = end_process - begin_process;




		///////////////////////// Decompose Image
			begin_process = getTime();
			
		    try{
				Stable = inc_decomp.decompose_image(image_cleaned, pixel_Tau, origin, map->info.resolution);
			}
			catch (...)  {			}

			
			end_process = getTime();	decompose_time = end_process - begin_process;
			
		////////////Draw Image & publish
		
			begin_process = getTime();
/*
	//		cv::Mat croppedRef(Colored_Frontier, resize_rect);			
			cv::flip(black_image, black_image,0);  cv::Mat big = Stable.draw_stable_contour() & ~black_image;

			cout << "Rect "<< first_rect << endl;

			big(first_rect).copyTo(grad);

//*/	
			
			grad = Stable.draw_stable_contour();	

			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			grad.convertTo(grad, CV_32F);
//			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_8UC1;			grad.convertTo(grad, CV_8UC1);
			grad.copyTo(cv_ptr->image);////most important

			end_process = getTime();	drawPublish_time = end_process - begin_process;
			whole_time = end_process - begin_whole;

			/////// Time Measures
			{
				printf("Time: total %.0f, Classified: occ %.1f, Decomp %.1f, Draw %.1f \n", whole_time, occupancy_time, decompose_time, drawPublish_time);
	
	
				clean_time_vector.push_back(occupancy_time);
				decomp_time_vector.push_back(decompose_time);
				paint_time_vector.push_back(drawPublish_time);
				complete_time_vector.push_back(whole_time);
				
				cout << "Time Vector size "<< clean_time_vector.size() << endl;
			}
			

/*
			for(int i=0; i < clean_time_vector.size(); i++){
//				cout << time_vector[i] << endl;
				printf("%.0f %.0f %.0f %.0f \n", paint_time_vector[i], clean_time_vector[i],  decomp_time_vector[i] , complete_time_vector[i]);
			}
			//*/
			
		/////////////////////////	
		}
			

/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  publish_Image();
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

		cv::Mat clean_image(cv::Mat Occ_Image, cv::Mat &black_image){
			//Occupancy Image to Free Space	
			
			cv::Mat valid_image = Occ_Image < 101;
			std::vector<std::vector<cv::Point> > test_contour;
			cv::findContours(valid_image, test_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

			cv::Rect first_rect = cv::boundingRect(test_contour[0]);
			for(int i=1; i < test_contour.size(); i++){
				first_rect |= cv::boundingRect(test_contour[i]);
			}
			cv::Mat reduced_Image;
			valid_image(first_rect).copyTo(reduced_Image);
			
			
			cv::Mat open_space = reduced_Image<10;
			black_image = reduced_Image>90 & reduced_Image<=100;		
			cv::Mat Median_Image, out_image, temp_image ;
			int filter_size=2;

			cv::boxFilter(black_image, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			black_image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// inflate obstacle

			filter_size=10;
			cv::boxFilter(open_space, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			Median_Image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			Median_Image = Median_Image | open_space ;
			//cv::medianBlur(Median_Image, Median_Image, 3);
			cv::dilate(Median_Image, Median_Image,cv::Mat());

			out_image = Median_Image & ~black_image;// Open space without obstacles




			cv::Size image_size = Occ_Image.size();
			cv::Mat image_out(image_size, CV_8UC1);
			cv::Mat black_image_out(image_size, CV_8UC1) ; 

			out_image.copyTo(image_out(first_rect));
			
			black_image.copyTo(black_image_out(first_rect));
			black_image =black_image_out;

			return image_out;
		}


		cv::Mat clean_image2(cv::Mat Occ_Image, cv::Mat &black_image){
			//Occupancy Image to Free Space	
			cv::Mat open_space = Occ_Image<10;
			black_image = Occ_Image>90 & Occ_Image<=100;		
			cv::Mat Median_Image, out_image, temp_image ;
			int filter_size=2;

			cv::boxFilter(black_image, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			black_image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// inflate obstacle

			filter_size=10;
			cv::boxFilter(open_space, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			Median_Image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			Median_Image = Median_Image | open_space ;
			//cv::medianBlur(Median_Image, Median_Image, 3);
			cv::dilate(Median_Image, Median_Image,cv::Mat());

			out_image = Median_Image & ~black_image;// Open space without obstacles

			return out_image;
		}

		cv::Rect find_image_bounding_Rect(cv::Mat Occ_Image){
			cv::Mat valid_image = Occ_Image < 101;
			std::vector<std::vector<cv::Point> > test_contour;
			cv::findContours(valid_image, test_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

			cv::Rect first_rect = cv::boundingRect(test_contour[0]);
			for(int i=1; i < test_contour.size(); i++){
				first_rect |= cv::boundingRect(test_contour[i]);
			}
			return first_rect;
		}

		void read_file(){
			cv::Mat image_GT, image_original;
//			image = cv::imread("image.png",0);   // Read the file
//			image = cv::imread("maps/Room_Segmentation/test_maps/Freiburg52_scan.png",0);   // Read the file
//			image = cv::imread("/home/unizar/ROS_Indigo/catkin_ws/src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan.png",0);   // Read the file
			image_original = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan.png",0);   // Read the file
			image_GT          = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps/Freiburg52_scan_gt_segmentation.png",0);   // Read the file
			
			if(! image_original.data )                              // Check for invalid input
		    {
		        cout <<  "Could not open or find the image" << std::endl ;
		    }
			
			std::cout << "Original:     (" <<  image_original.rows <<" , "<<image_original.cols<<")" << std::endl;
			std::cout << "Ground Truth: (" <<  image_GT.rows <<" , "<<image_GT.cols<<")" << std::endl;

			cv::Point2f origin(0,0);

		/////////////////////////////////////// Decompose GT
			cv::Mat pre_decompose = image_GT.clone();
			cv::Mat pre_decompose_BW = pre_decompose > 250;

			Stable = inc_decomp.decompose_image(pre_decompose_BW, 3.5, origin , 0.05);

			cv::Mat GT_segmentation = Stable.draw_stable_contour();	
		//////////////////////////////////////// Decompose Original
			Incremental_Decomposer inc_decomp2;
			Stable_graph Stable2;

			cv::Mat pre_decompose2 = image_original.clone() ;			
			cv::Mat pre_decompose2_BW = pre_decompose2 > 250;			

			Stable2 = inc_decomp2.decompose_image(pre_decompose2_BW, 3.5, origin , 0.05);

			cv::Mat DuDe_segmentation = Stable2.draw_stable_contour();	
			
			std::cout << "Decomposition DuDe     has "<< Stable2.Region_contour.size() << " tags"<<std::endl;
			std::cout << "Decomposition Original has "<< Stable .Region_contour.size() << " tags"<<std::endl;
		/////////////////////////////////////

//*

			match2points relation2points;
			std::set <int> GT_tags, DuDe_tags;
			for(int x=0; x < image_original.rows; x++){
				for(int y=0; y < image_original.cols; y++){
					cv::Point current_point(x,y);
					std::vector < int > relation;
					
					int GT_tag   = GT_segmentation.  at<uchar>(y,x);
					int DuDe_tag = DuDe_segmentation.at<uchar>(y,x);
					
					GT_tags.insert(GT_tag); 
					DuDe_tags.insert(DuDe_tag);
					
					relation.push_back( GT_tag );
					relation.push_back( DuDe_tag );

					relation2points[relation].push_back(current_point);
					
//					std::cout << "GT Tags "<< GT_tag << std::endl;
					
				}
			}
			
			for( match2points::iterator it = relation2points.begin(); it!= relation2points.end(); it++ ){
				std::vector < int > current_relation = it->first;
				std::vector < cv::Point > current_points = it->second;
//				std::cout << "Relation ("<< current_relation[0] << "," << current_relation[1] << ") with " << current_points.size() << " points" << std::endl;
			}
/*
			for( std::set <int>::iterator it = GT_tags.begin(); it!= GT_tags.end(); it++ ){
				std::cout << "GT Tags "<< *it << std::endl;
			}
			for( std::set <int>::iterator it = DuDe_tags.begin(); it!= DuDe_tags.end(); it++ ){
				std::cout << "DuDe tags "<< *it << std::endl;
			}
//*/

			/////////////
			cv::Mat to_publish = GT_segmentation;
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish.convertTo(to_publish, CV_32F);
			to_publish.copyTo(cv_ptr->image);////most important
			////////////
			cv::Mat to_publish2 = pre_decompose2_BW;
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish2, CV_32F);
			to_publish2.copyTo(cv_ptr2->image);////most important
			///////////
			

		}


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

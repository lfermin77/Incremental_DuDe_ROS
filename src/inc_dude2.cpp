//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"


//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//DuDe
#include "inc_decomp.hpp"





class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Subscriber graph_sub_;	
	ros::Subscriber Uncertainty_sub_;
    ros::Publisher  pose_array_pub_;	
	ros::Timer timer;
			
	float Decomp_threshold_;
	Incremental_Decomposer inc_decomp;
	Stable_graph Stable;

	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;

	ros::Subscriber trajectory_sub_;
	
	std::vector<geometry_msgs::Point> edges;
	
	geometry_msgs::Point Last_node;
	
	public:
		ROS_handler(const std::string& mapname, float threshold) : mapname_(mapname),  it_(n), Decomp_threshold_(threshold)
		{
			ROS_INFO("Waiting for the map");
			map_sub_ = n.subscribe("map", 2, &ROS_handler::mapCallback, this); //mapname_ to include different name
			trajectory_sub_ = n.subscribe("trajectory", 1, &ROS_handler::trajectoryCallback, this);
			
			graph_sub_ = n.subscribe("SLAM_Graph", 10, &ROS_handler::graphCallback, this);
			
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/image_frontier", 1);
			
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
			
			Uncertainty_sub_ = n.subscribe("query_Uncertainty", 10, &ROS_handler::UncertaintyCallback, this);
			pose_array_pub_  = n.advertise<geometry_msgs::PoseArray>("query_Poses", 10);
			
		}

		~ROS_handler()	{
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

//*			


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

			printf("Time: total %.0f, Classified: occ %.1f, Decomp %.1f, Draw %.1f \n", whole_time, occupancy_time, decompose_time, drawPublish_time);


			clean_time_vector.push_back(occupancy_time);
			decomp_time_vector.push_back(decompose_time);
			paint_time_vector.push_back(drawPublish_time);
			complete_time_vector.push_back(whole_time);
			
			cout << "Time Vector size "<< clean_time_vector.size() << endl;
			
			
			
			
			
			
			begin_process = getTime();
//			graph_iteration();
			end_process = getTime();	drawPublish_time = end_process - begin_process;			
			printf("Time for edges:  %.3f \n", drawPublish_time);





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


////////////////
		void trajectoryCallback(const geometry_msgs::PoseArray &msg)
		{
			/*
			for(int i=0; i< msg.poses.size();i++){
				std::cout << "Pose is "<< msg.poses[i].position.x  <<std::endl;
			}//*/
			Last_node = msg.poses.front().position;
		}

////////////////
		void graphCallback(const visualization_msgs::Marker& graph_msg)
		{
			edges = graph_msg.points;

		}

///////////////
		void UncertaintyCallback(const geometry_msgs::PoseWithCovariance& msg)
		{
			int a=1;
		}


////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
		}



/////////////////////////
//// UTILITY
/////////////////////////

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

						
typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;
		void find_contour_connectivity_and_frontier(cv::Mat  Tag_image, cv::Mat  original_image){
			
//			UtilityGraph Region_Graph;
			int window_size=3;
			

			edge_points_mapper mapping_set_to_point_array, mapping_frontier_to_point_array;
			for (int i=window_size;i < Tag_image.size().width- window_size ;i++){
				for (int j=window_size;j < Tag_image.size().height - window_size ;j++){
				
					/////////////////////
					cv::Point window_center(i,j);
					
					std::set<int>  connections_in_region, frontier_connections;
					for(int x=-window_size; x <= window_size; x++){
						for(int y=-window_size; y <= window_size; y++){
							cv::Point delta(x,y); 							
							cv::Point current_point = window_center + delta;
							int tag = Tag_image.at<uchar>(current_point);
							int frontier = original_image.at<uchar>(current_point);
							
							if (tag>0){
								connections_in_region.insert( tag -1 );
								frontier_connections.insert( tag -1 );
							}
							if ( frontier==255) frontier_connections.insert( -1 );

						}
					}
					//////////////////
				if(connections_in_region.size()>1){					
					mapping_set_to_point_array[connections_in_region].push_back(window_center);
				}
				if(frontier_connections.size()>1 &&  ( (*frontier_connections.begin())==-1)  ){					
					mapping_frontier_to_point_array[frontier_connections].push_back(window_center);
				}
				}
			}
			//////////////	
			
			
//*
			for (edge_points_mapper::iterator it2 = mapping_set_to_point_array.begin(); it2 != mapping_set_to_point_array.end(); it2 ++){

				std::cout << "Connections  are: ";
				std::set<int> current_connections_set = it2->first ;
				for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
					std::cout <<" " << *it;
				}
				
				std::vector<cv::Point> current_points = it2->second;
				cv::Point average_point(0,0);

				for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
					average_point += *it;
				}		
				std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ")";
				
				std::cout << std::endl;
			}
			//*/
			
//*
			for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){

				std::cout << "Frontiers  are: ";
				std::set<int> current_connections_set = it2->first ;
				for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
					std::cout <<" " << *it;
				}
				
				std::vector<cv::Point> current_points = it2->second;
				cv::Point average_point(0,0);

				for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
					average_point += *it;
				}		
				std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ")";
				
				std::cout << std::endl;
			}
			//*/



				

		}

		void are_contours_connected(vector<cv::Point> first_contour, vector<cv::Point> second_contour, cv::Point &centroid, int &number_of_ones ){
			
			vector< cv::Point > closer_point;
			cv::Point acum(0,0);
			int threshold=2;
			
			for(int i=0; i<first_contour.size();i++){
				for(int j=0; j< second_contour.size();j++){
					float distance;
					distance = cv::norm(first_contour[i] -  second_contour[j] );
					if(distance < threshold){
						cv::Point point_to_add;
						point_to_add.x = (first_contour[i].x + second_contour[j].x)/2;
						point_to_add.y = (first_contour[i].y + second_contour[j].y)/2;
						
						closer_point.push_back(point_to_add);
						acum += point_to_add;						
					 }					
				}
			}

			number_of_ones = closer_point.size();
			centroid.x = acum.x/number_of_ones;
			centroid.y = acum.y/number_of_ones;
		}



};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Exploration_Fraenkel");
	
	std::string mapname = "map";
	
	float decomp_th=3;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(mapname, decomp_th);
	ros::spin();
	
	return 0;
}

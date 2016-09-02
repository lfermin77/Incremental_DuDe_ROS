//ROS
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Twist.h"

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

//DuDe
#include "inc_decomp.hpp"


//cpp
#include <dirent.h>


class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_, it2_;
	image_transport::Subscriber image_sub_, image_sub2_;
	image_transport::Publisher image_pub_, image_pub2_;	
	cv_bridge::CvImagePtr cv_ptr, cv_ptr2;
		
	ros::Timer timer;
	ros::Subscriber twist_sub_;	
			
	float Decomp_threshold_;
	bool segmentation_ready;
	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;
	
	std::string  base_path;
	std::string gt_ending;
	std::string FuT_ending;
	
	std::vector < std::vector<float> > Precisions;
	std::vector < std::vector<float> > Recalls;
	std::vector<double> Times;

	int current_file;
	
	public:
		ROS_handler( float threshold) :   it_(n), it2_(n), Decomp_threshold_(threshold)
		{
			ROS_INFO("Waiting for the map");
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			twist_sub_ = n.subscribe("cmd_vel", 2, &ROS_handler::twistCallback, this);
			segmentation_ready = false;
			
			image_pub_  = it_.advertise("/ground_truth_segmentation", 1);			
			image_pub2_ = it_.advertise("/DuDe_segmentation",   1);			

			cv_ptr.reset (new cv_bridge::CvImage);
			//cv_ptr->encoding = "mono8";
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

			cv_ptr2.reset (new cv_bridge::CvImage);
			//cv_ptr2->encoding = "mono8";
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			
//			base_path = "src/Incremental_DuDe_ROS/maps/Room_Segmentation/test_maps";
//			base_path = "src/Incremental_DuDe_ROS/maps/Room_Segmentation/nested_maps";
			base_path = "src/Incremental_DuDe_ROS/maps/Room_Segmentation/all_maps";
//			base_path = "src/Incremental_DuDe_ROS/maps/Room_Segmentation/bad_maps";
			gt_ending = "_gt_segmentation.png";
			FuT_ending ="_furnitures.png";

			current_file=0;
			init();			//read_files();
						
		}


/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  if(segmentation_ready) publish_Image();
		}


		void twistCallback(const geometry_msgs::Twist& msg)
		{
//		  ROS_INFO("tic tac");
			std::vector<std::string> files_to_read = listFile();
			

//			std::cout << "Files listed  " << std::endl;			
			if(current_file < files_to_read.size()){
				std::cout << "Reading file  "<< files_to_read[current_file] << std::endl<< std::endl;
				read_files(files_to_read[current_file]);

				
				std::cout << "Processed file  "<< files_to_read[current_file] << std::endl<< std::endl;
				//Print results
				std::cout << "Results  " << std::endl;
				float cum_precision=0;
				float cum_recall=0;
				int size_precision=0, size_recall=0;
				for(int i=0; i < Precisions.size();i++){
					for(int j=0; j < Precisions[i].size();j++){
						cum_precision += Precisions[i][j];
						size_precision++;
					}
				}			
				for(int i=0; i < Recalls.size();i++){
					for(int j=0; j < Recalls[i].size();j++){
						cum_recall    += Recalls[i][j];
						size_recall++;
					}
				}			
				std::cout << "Average Precision: "<<  cum_precision/size_precision  << ", Average Recall: "<<  cum_recall/size_recall << std::endl;
				current_file++;
			}
			
			else{
				std::cout << "All files processed " << std::endl;
				std::cout << "Final Results " << std::endl;
				float cum_precision=0;
				float cum_recall=0;
				int size_precision=0, size_recall=0;
				for(int i=0; i < Precisions.size();i++){
					for(int j=0; j < Precisions[i].size();j++){
						cum_precision += Precisions[i][j];
						size_precision++;
					}
				}			
				for(int i=0; i < Recalls.size();i++){
					for(int j=0; j < Recalls[i].size();j++){
						cum_recall    += Recalls[i][j];
						size_recall++;
					}
				}			
				std::cout << "Average Precision: "<<  cum_precision/size_precision  << ", Average Recall: "<<  cum_recall/size_recall << std::endl;
			}
						
		}



		void twistCallback2(const geometry_msgs::Twist& msg)
		{
//			cv::Mat image_GT_BW     = cv::imread("src/Incremental_DuDe_ROS/maps/Room_Segmentation/nested_maps/lab_intel_gt_segmentation.png",0);   // Read the file
//			cv::Mat image_GT_tagged = segment_Ground_Truth(image_GT_BW);
			
			
			std::vector<std::string> files_to_read = listFile();
			

//			std::cout << "Files listed  " << std::endl;			
			if(current_file < files_to_read.size()){
				std::cout << "Reading file  "<< files_to_read[current_file] << std::endl<< std::endl;

				std::string full_path_original = base_path + "/" + files_to_read[current_file] + gt_ending;
				cv::Mat image_GT_BW    = cv::imread(full_path_original,0);   // Read the file
				cv::Mat image_GT_tagged = segment_Ground_Truth(image_GT_BW);

				
				/////////////
				cv::Mat to_publish = (image_GT_BW > 250);
				cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish.convertTo(to_publish, CV_32F);
				to_publish.copyTo(cv_ptr->image);////most important
				////////////
				cv::Mat to_publish2 = image_GT_tagged.clone();
				cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish2, CV_32F);
				to_publish2.copyTo(cv_ptr2->image);////most important
				///////////
				
				segmentation_ready = true;
				current_file++;
			}
//*/			
			
			
			
			
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
		typedef std::map <int, tag2points> tag2tagMapper;
	//////////////////////
		void read_files(std::string name){
			cv::Mat image_GT, image_original;

			std::string full_path_GT       = base_path + "/" + name + gt_ending;
			std::string full_path_original = base_path + "/" + name + FuT_ending;

			image_original    = cv::imread(full_path_original,0);   // Read the file
			image_GT          = cv::imread(full_path_GT,0);   // Read the file


			cv::Mat GT_segmentation = segment_Ground_Truth(image_GT);
//			cv::Mat GT_segmentation = simple_segment(image_GT);

			double begin_process, end_process, decompose_time;
			begin_process = getTime();
			
			cv::Mat DuDe_segmentation = simple_segment(image_original);

			end_process = getTime();	decompose_time = end_process - begin_process;			
			std::cout << "Time to decompose " << decompose_time << std::endl;
			Times.push_back(decompose_time);
			
			std::string full_path_decomposed       = base_path + "/" + name + "_DuDe_segmented.png";
//			cv::imwrite( full_path_decomposed , DuDe_segmentation );
			

			compare_images(GT_segmentation, DuDe_segmentation);
			
						
			/////////////
			cv::Mat to_publish = GT_segmentation.clone();
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish.convertTo(to_publish, CV_32F);
			to_publish.copyTo(cv_ptr->image);////most important
			////////////
			cv::Mat to_publish2 = DuDe_segmentation.clone();
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish2, CV_32F);
			to_publish2.copyTo(cv_ptr2->image);////most important
			///////////
			
			segmentation_ready = true;

		}

	/////////////////////
		void compare_images(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in){
			
			cv::Mat GT_segmentation   = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
			cv::Mat DuDe_segmentation = cv::Mat::zeros(GT_segmentation_in.size(),CV_8UC1);
			
			GT_segmentation_in  .convertTo(GT_segmentation, CV_8UC1);
			DuDe_segmentation_in.convertTo(DuDe_segmentation, CV_8UC1);			
			tag2tagMapper gt_tag2mapper,DuDe_tag2mapper;
			
			for(int x=0; x < GT_segmentation.size().width; x++){
				for(int y=0; y < GT_segmentation.size().height; y++){
					cv::Point current_pixel(x,y);
										
					int tag_GT   = GT_segmentation.at<uchar>(current_pixel);
					int tag_DuDe  = DuDe_segmentation.at<uchar>(current_pixel);
					
					if(tag_DuDe>0 && tag_GT>0 ){
						gt_tag2mapper  [tag_GT][tag_DuDe].push_back(current_pixel);
						DuDe_tag2mapper[tag_DuDe][tag_GT].push_back(current_pixel);
					}
				}
			}
			
			std::vector<float> precisions_inside, recalls_inside;			

//			std::cout << "Regions in GT: "<< std::endl;
			for( tag2tagMapper::iterator it = gt_tag2mapper.begin(); it!= gt_tag2mapper.end(); it++ ){
//				std::cout << "   " << it->first << " connected to "<< std::endl;
				tag2points inside = it->second;
				int max_intersection=0, total_points=0; 
				for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
					total_points += it2->second.size();
					if (it2->second.size() > max_intersection) max_intersection = it2->second.size();
//					std::cout << "      " << it2->first << " with "<< it2->second.size() <<" points" << std::endl;					
				}
//				std::cout << "   max is " << max_intersection << " that represents " << 100*max_intersection/total_points   << std::endl;
				precisions_inside.push_back(100*max_intersection/total_points);
			}			
			
//			std::cout << "Regions in DuDe: "<< std::endl;
			for( tag2tagMapper::iterator it = DuDe_tag2mapper.begin(); it!= DuDe_tag2mapper.end(); it++ ){
//				std::cout << "   " << it->first << " connected to "<< std::endl;
				tag2points inside = it->second;
				int max_intersection=0, total_points=0; 
				for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
					total_points += it2->second.size();
					if (it2->second.size() > max_intersection) max_intersection = it2->second.size();
//					std::cout << "      " << it2->first << " with "<< it2->second.size() <<" points" << std::endl;					
				}
//				std::cout << "   max is " << max_intersection << " that represents " << 100*max_intersection/total_points   << std::endl;
				recalls_inside.push_back(100*max_intersection/total_points);
			}			
			

			Precisions.push_back(precisions_inside);
			Recalls.push_back   (recalls_inside);
			
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
		cv::Mat segment_Ground_Truth(cv::Mat GroundTruth_BW){
			cv::Mat src = GroundTruth_BW.clone();
			cv::Mat drawing = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

			src = src > 250;
			
			cv::erode(src, src, cv::Mat(), cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// erode ground truth obstacle
			
			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			
			cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
			
			// iterate through all the top-level contours,
			// draw each connected component with its own random color
			int idx = 0;
			int color=1;
			for( ; idx >= 0; idx = hierarchy[idx][0] )
			{
//				cv::drawContours( drawing, contours, idx, (rand()%244 + 10) , CV_FILLED, 20, hierarchy );
				cv::drawContours( drawing, contours, idx, color , CV_FILLED, 20, hierarchy );
				color++;
			}
			cv::dilate(drawing, drawing, cv::Mat(), cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// erode dilate drawing
			return drawing;
		}

	////////////////////
		std::vector<std::string> listFile(){
	        DIR *pDIR;
	        struct dirent *entry;
			std::vector<std::string> files_to_read;

	        if( pDIR=opendir(base_path.c_str()) ){
				while(entry = readdir(pDIR)){
					if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 ){
						std::string const fullString = entry->d_name;

						if (fullString.length() >= gt_ending.length()) {
							if (0 == fullString.compare (fullString.length() - gt_ending.length(), gt_ending.length(), gt_ending)){
								std::string filename= fullString.substr (0,fullString.length() - gt_ending.length());
								files_to_read.push_back(filename);
//								std::cout << filename << "\n";
							}
						}
					}
				}
				closedir(pDIR);
	        }
			std::cout << "Files size "<< files_to_read.size() << std::endl;
			return files_to_read;
		}
	////////////////////
		void init(){
			std::vector<std::string> files_to_read = listFile();
			
//			read_files("Freiburg101");
//			read_files(files_to_read[7]);
			
			std::cout << "Files listed  " << std::endl;			
			for (int i=0; i < files_to_read.size() ; i++){
				std::cout << "Reading file  "<< files_to_read[i] << std::endl<< std::endl<< std::endl<< std::endl;
				read_files(files_to_read[i]);
				publish_Image();
			}
			
			print_report(files_to_read);

			
		}

	///////////////////
		void print_report(std::vector<std::string> files_to_read){
			
			
			//Print results
			std::cout << "Final Results  " << std::endl;
			double cum_time=0;
			
			for(int file_num=0; file_num < files_to_read.size(); file_num++){
				//Print results
				std::cout << "   "<< file_num+1 << ") File:  " << files_to_read[file_num] << std::endl;
				float cum_precision=0;
				float cum_recall=0;
				int size_precision=0, size_recall=0;
				for(int j=0; j < Precisions[file_num].size();j++){
					cum_precision += Precisions[file_num][j];
					size_precision++;
				}
				for(int j=0; j < Recalls[file_num].size();j++){
					cum_recall    += Recalls[file_num][j];
					size_recall++;
				}
				printf("     Avg Precision: %.2f, Avg Recall: %.2f, time: %.3f \n",cum_precision/size_precision, cum_recall/size_recall, Times[file_num]/1000);
				current_file++;
				cum_time += Times[file_num];
			}
			
			
			
			
			
			float cum_precision=0;
			float cum_recall=0;
			int size_precision=0, size_recall=0;
			for(int i=0; i < Precisions.size();i++){
				for(int j=0; j < Precisions[i].size();j++){
					cum_precision += Precisions[i][j];
					size_precision++;
				}
			}			
			for(int i=0; i < Recalls.size();i++){
				for(int j=0; j < Recalls[i].size();j++){
					cum_recall    += Recalls[i][j];
					size_recall++;
				}
			}			

//			std::cout << "Full Average Precision: "<<  cum_precision/size_precision  << ", Full Average Recall: "<<  cum_recall/size_recall << std::endl;
			std::cout << files_to_read.size();
			printf(" files processed. Avg time: %.3f, Avg Precision: %.2f%%, Avg Recall %.2f%% \n", (cum_time/files_to_read.size() )/1000, cum_precision/size_precision, cum_recall/size_recall);

//			std::cout << "Full Average Precision: "<<  cum_precision/size_precision  << ", Full Average Recall: "<<  cum_recall/size_recall << std::endl;
		}

};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "evaluation");
	
	
	float decomp_th=3;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(decomp_th);
	ros::spin();
	
	return 0;
}

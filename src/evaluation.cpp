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

struct results{
	double time;
	double precision;
	double recall;
};





class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_, it2_, it3_;
	image_transport::Subscriber image_sub_, image_sub2_, image_sub3_;
	image_transport::Publisher image_pub_, image_pub2_, image_pub3_;	
	cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
		
	ros::Timer timer;
	ros::Subscriber twist_sub_;	
			
	float Decomp_threshold_;
	bool segmentation_ready;
	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;
	
	std::string  base_path;
	std::string gt_ending;
	std::string FuT_ending;
	std::string No_FuT_ending; 
	
	std::vector < std::vector<float> > Precisions;
	std::vector < std::vector<float> > Recalls;
	std::vector<double> Times;

	int current_file;
	std::set<std::string>  file_list;
	std::set<std::string>::iterator file_it;
	
	double pixel_precision, pixel_recall;
	
	public:
		ROS_handler( float threshold) :   it_(n), it2_(n), it3_(n), Decomp_threshold_(threshold)
		{
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			twist_sub_ = n.subscribe("cmd_vel", 1, &ROS_handler::twistCallback, this);
			segmentation_ready = false;
			
			image_pub_  = it_.advertise("/ground_truth_segmentation", 1);			
			image_pub2_ = it_.advertise("/DuDe_segmentation",   1);			
			image_pub3_ = it_.advertise("/Inc_DuDe_segmentation",   1);			

			cv_ptr.reset (new cv_bridge::CvImage);
			//cv_ptr->encoding = "mono8";
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

			cv_ptr2.reset (new cv_bridge::CvImage);
			//cv_ptr2->encoding = "mono8";
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

			cv_ptr3.reset (new cv_bridge::CvImage);
			//cv_ptr3->encoding = "mono8";
			cv_ptr3->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			

			base_path = "src/Incremental_DuDe_ROS/maps/Room_Segmentation/all_maps";
			gt_ending = "_gt_segmentation.png";
				/// With    furniture
			FuT_ending ="_furnitures.png";
				/// Without furniture
			No_FuT_ending =".png"; 

			current_file=0;
			file_list = listFile();
			file_it = file_list.begin();
			

			std::cout << "File to process:  "<< *file_it << std::endl<< std::endl;
//			process_all_files();
					
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
			float direction = msg.linear.x;
			
			if(direction >0){
				file_it++;
				if(file_it == file_list.end() ) file_it = file_list.begin();
				
				std::cout << "File to process:  "<< *file_it << std::endl<< std::endl;
			}
			else{
							
				std::cout << "Processing file  "<< *file_it << std::endl<< std::endl;
				Precisions.clear();			Recalls.clear();
				
				process_files(*file_it);
//				process_files_incrementally(*file_it);

				
				std::cout << "Processed file  "<< *file_it << std::endl<< std::endl;


				publish_Image();
			}
			

		}





////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish (cv_ptr->toImageMsg());
			image_pub2_.publish(cv_ptr2->toImageMsg());
			image_pub3_.publish(cv_ptr3->toImageMsg());
		}



/////////////////////////
//// UTILITY
/////////////////////////

		typedef std::map <std::vector<int>, std::vector <cv::Point> > match2points;
		typedef std::map <int, std::vector <cv::Point> > tag2points;
		typedef std::map <int, tag2points> tag2tagMapper;

	//////////////////////
		void process_files(std::string name){
			cv::Mat image_GT, image_Furniture, image_No_Furniture;
			cv::Mat DuDe_Furniture, DuDe_No_Furniture;
			cv::Mat proxy, zero_image;

			std::string full_path_GT           = base_path + "/" + name + gt_ending;
			std::string full_path_Furniture    = base_path + "/" + name + FuT_ending;
			std::string full_path_No_Furniture = base_path + "/" + name + No_FuT_ending;
			std::string saving_path            = base_path + "/Tagged_Images/" + name;

			double begin_process, end_process, decompose_time;
			std::vector <cv::Vec3b> colormap;
			results No_Furn_Results_pixel, No_Furn_Results_Regions;
			results Furn_Results_pixel, Furn_Results_Regions;
			



			image_GT              = cv::imread(full_path_GT,0);   // Read the file
			image_Furniture       = cv::imread(full_path_Furniture,0);   // Read the file
			image_No_Furniture    = cv::imread(full_path_No_Furniture,0);   // Read the file
			zero_image =cv::Mat::zeros(image_GT.size(),CV_8U);
			
			///////// Original
			cv::Mat GT_segmentation = segment_Ground_Truth(image_GT);			
			colormap = save_image_original_color(saving_path + "_TAG" + gt_ending, GT_segmentation);
			
			
			
			////////No Furniture
			begin_process = getTime();	
				DuDe_No_Furniture = simple_segment(image_No_Furniture);
			end_process = getTime();	decompose_time = end_process - begin_process;			
			std::map<int,int> DuDe_NoF_map = compare_images(GT_segmentation, DuDe_No_Furniture);			

			DuDe_No_Furniture.copyTo( proxy ,image_No_Furniture>250);						
			DuDe_No_Furniture = proxy.clone(); 
//			save_decomposed_image_color(saving_path + "_DuDe" + No_FuT_ending, DuDe_No_Furniture, colormap, DuDe_NoF_map); proxy = zero_image;

			No_Furn_Results_pixel.time = No_Furn_Results_Regions.time = decompose_time;
			extract_results(No_Furn_Results_pixel, No_Furn_Results_Regions);


			////////Furniture
			begin_process = getTime();	
				DuDe_Furniture = simple_segment(image_Furniture);
			end_process = getTime();	decompose_time = end_process - begin_process;			
			std::map<int,int> DuDe_Furn_map = compare_images(GT_segmentation, DuDe_Furniture);

			DuDe_Furniture.copyTo( proxy ,image_Furniture>250);						
			DuDe_Furniture = proxy.clone(); 
//			save_decomposed_image_color(saving_path + "_DuDe" + FuT_ending, DuDe_Furniture, colormap, DuDe_Furn_map);  proxy = zero_image;

			Furn_Results_pixel.time = Furn_Results_Regions.time = decompose_time;
			extract_results(Furn_Results_pixel, Furn_Results_Regions);


			
			///////////////////////////
			double min, max;
			cv::minMaxLoc(GT_segmentation,&min,&max);
			
			float rows = GT_segmentation.rows;
			float cols = GT_segmentation.cols;
			float proper_size = rows*cols/1000;
			proper_size = proper_size/1000;


			std::cout << name;
			printf(" No_Furniture Precision: %.1f Recall: %.1f time: %.0f Labels %.0f size %.2f\n",No_Furn_Results_Regions.precision, No_Furn_Results_Regions.recall, No_Furn_Results_Regions.time, max, proper_size);

			std::cout << name;
			printf(" Furniture Precision: %.1f Recall: %.1f time: %.0f Labels %.0f size %.2f\n",Furn_Results_Regions.precision, Furn_Results_Regions.recall, Furn_Results_Regions.time, max, proper_size);





			

			/////////////
			cv::Mat to_publish = image_GT.clone();
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish.convertTo(to_publish, CV_32F);
			to_publish.copyTo(cv_ptr->image);////most important
			////////////
			cv::Mat to_publish2 = image_Furniture.clone();
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish2, CV_32F);
			to_publish2.copyTo(cv_ptr2->image);////most important
			///////////
			cv::Mat to_publish3 = image_No_Furniture.clone();
			cv_ptr3->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish3, CV_32F);
			to_publish3.copyTo(cv_ptr3->image);////most important
			///////////
			
			segmentation_ready = true;

		}

	//////////////////////
		void process_files_incrementally(std::string name){
			cv::Mat image_GT, image_Furniture, image_No_Furniture;
			cv::Mat Inc_Furniture, Inc_No_Furniture;
			cv::Mat proxy, zero_image;

			std::string full_path_GT           = base_path + "/" + name + gt_ending;
			std::string full_path_Furniture    = base_path + "/" + name + FuT_ending;
			std::string full_path_No_Furniture = base_path + "/" + name + No_FuT_ending;
			std::string saving_path       = base_path + "/Tagged_Images/" + name;

			double begin_process, end_process, decompose_time;
			std::vector <cv::Vec3b> colormap;
			results No_Furn_Results_pixel, No_Furn_Results_Regions;
			results Furn_Results_pixel, Furn_Results_Regions;
			


			image_GT              = cv::imread(full_path_GT,0);   // Read the file
			image_Furniture       = cv::imread(full_path_Furniture,0);   // Read the file
			image_No_Furniture    = cv::imread(full_path_No_Furniture,0);   // Read the file
			zero_image =cv::Mat::zeros(image_GT.size(),CV_8U);
			
			///////// Original
			cv::Mat GT_segmentation = segment_Ground_Truth(image_GT);			
			colormap = save_image_original_color(saving_path + "_TAG_inc" + gt_ending, GT_segmentation);
			
			
			
			////////No Furniture
				Inc_No_Furniture = incremental_segment(image_No_Furniture, decompose_time);
			std::map<int,int> DuDe_NoF_map = compare_images(GT_segmentation, Inc_No_Furniture);			

			Inc_No_Furniture.copyTo( proxy ,image_No_Furniture>250);						
			Inc_No_Furniture = proxy.clone(); 
			save_decomposed_image_color(saving_path + "_Inc" + No_FuT_ending, Inc_No_Furniture, colormap, DuDe_NoF_map); proxy = zero_image;

			No_Furn_Results_pixel.time = No_Furn_Results_Regions.time = decompose_time;
			extract_results(No_Furn_Results_pixel, No_Furn_Results_Regions);


			////////Furniture
				Inc_Furniture = incremental_segment(image_No_Furniture, decompose_time);
			std::map<int,int> DuDe_Furn_map = compare_images(GT_segmentation, Inc_Furniture);

			Inc_Furniture.copyTo( proxy ,image_Furniture>250);						
			Inc_Furniture = proxy.clone(); 
			save_decomposed_image_color(saving_path + "_Inc" + FuT_ending, Inc_Furniture, colormap, DuDe_Furn_map);  proxy = zero_image;

			Furn_Results_pixel.time = Furn_Results_Regions.time = decompose_time;
			extract_results(Furn_Results_pixel, Furn_Results_Regions);
			
			///////////////////////////


			std::cout << name;
			printf(" No_Furniture Precision: %.1f Recall: %.1f time: %.0f \n",No_Furn_Results_pixel.precision, No_Furn_Results_pixel.recall, No_Furn_Results_pixel.time);

			std::cout << name;
			printf(" Furniture Precision: %.1f Recall: %.1f time: %.0f \n",Furn_Results_pixel.precision, Furn_Results_pixel.recall, Furn_Results_pixel.time);





			

			/////////////
			cv::Mat to_publish = image_GT.clone();
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish.convertTo(to_publish, CV_32F);
			to_publish.copyTo(cv_ptr->image);////most important
			////////////
			cv::Mat to_publish2 = image_Furniture.clone();
			cv_ptr2->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish2, CV_32F);
			to_publish2.copyTo(cv_ptr2->image);////most important
			///////////
			cv::Mat to_publish3 = image_No_Furniture.clone();
			cv_ptr3->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			to_publish2.convertTo(to_publish3, CV_32F);
			to_publish3.copyTo(cv_ptr3->image);////most important
			///////////
			
			segmentation_ready = true;

		}



	/////////////////////
		std::map<int,int> compare_images(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in){
			
			std::map<int,int> segmented2GT_tags;
			
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
			double cum_precision=0, cum_total=0, cum_recall=0;

//			std::cout << "Regions in GT: "<< std::endl;
			for( tag2tagMapper::iterator it = gt_tag2mapper.begin(); it!= gt_tag2mapper.end(); it++ ){
//				std::cout << "   " << it->first << " connected to "<< std::endl;
				tag2points inside = it->second;
				int max_intersection=0, total_points=0; 
				int gt_tag_max = -1;
				for( tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
					total_points += it2->second.size();
					if (it2->second.size() > max_intersection){
						max_intersection = it2->second.size();
						gt_tag_max = it2->first;
					}
//					std::cout << "      " << it2->first << " with "<< it2->second.size() <<" points" << std::endl;					
				}
				segmented2GT_tags[gt_tag_max] = it->first;
//				std::cout << "   max is " << max_intersection << " that represents " << 100*max_intersection/total_points   << std::endl;
				precisions_inside.push_back(100*max_intersection/total_points);
				cum_precision += max_intersection;
				cum_total += total_points;
			}	
			pixel_precision = cum_precision/cum_total;
					
			cum_total=0;
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
				cum_recall += max_intersection;
				cum_total += total_points;
			}			
			pixel_recall=cum_recall/cum_total;

			Precisions.push_back(precisions_inside);
			Recalls.push_back   (recalls_inside);
			return(segmented2GT_tags);
		}

	////////////////////
		cv::Mat simple_segment(cv::Mat image_in){
			Incremental_Decomposer inc_decomp;
			Stable_graph Stable;
			cv::Point2f origin(0,0);
			float resolution = 0.05;

			
			cv::Mat pre_decompose = image_in.clone();
			cv::Mat pre_decompose_BW = pre_decompose > 250;
//			cv::Mat pre_decompose_BW = clean_image(pre_decompose > 250);


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
				cv::drawContours( drawing, contours, idx, color , CV_FILLED, 20, hierarchy );
				color++;
			
			}
			cv::dilate(drawing, drawing, cv::Mat(), cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// erode dilate drawing
			return drawing;
		}

	////////////////////
		cv::Mat incremental_segment(cv::Mat image_in, double & time){
			Incremental_Decomposer inc_decomp;
			Stable_graph Stable;
			cv::Point2f origin(0,0);
			float resolution = 0.05;

			
			cv::Mat pre_decompose = image_in.clone();
			cv::Mat pre_decompose_BW = pre_decompose > 250;
//			cv::Mat pre_decompose_BW = clean_image(pre_decompose > 250);



			cv::Mat current_circle = cv::Mat::zeros(image_in.size(),CV_8UC1);
			cv::Mat scanned_image = cv::Mat::zeros(image_in.size(),CV_8UC1);
			cv::Mat current_scan;
							
			int counter=0;
			bool stop_criteria=false;
			float cum_time=0;
			int valid_images=0;

			while(!stop_criteria){
				div_t divresult = div (50*counter,pre_decompose_BW.size().width );
				int x = divresult.rem;
				int y = 50*divresult.quot;
	
				cv::Point current_position = cv::Point(x,y);
				cv::circle(current_circle, current_position, 100, 1, -1);

				pre_decompose_BW.copyTo(current_scan, current_circle); // Aggregated Scan		

				///////////////
				if (countNonZero(current_scan)  > 160) {
					double begin_process, end_process, decompose_time;
					begin_process = getTime();						
					Stable = inc_decomp.decompose_image(current_scan, Decomp_threshold_/resolution, origin , resolution);				
					end_process = getTime();	decompose_time = end_process - begin_process;			
					valid_images++;
					cum_time += decompose_time;
				}
					
				////////////////

				
								
				if( (x > pre_decompose_BW.size().width) || (y > pre_decompose_BW.size().height) ) {
					stop_criteria = true;
				}
				scanned_image |= current_scan;
				counter++;
			}

			///////////
			cv::Mat Drawing = cv::Mat::zeros(image_in.size(), CV_8UC1);	
			for(int i = 0; i < Stable.Region_contour.size();i++){
				drawContours(Drawing, Stable.Region_contour, i, i+1, -1, 8);
			}
			time = cum_time/valid_images;
			std::cout << "Decomposition size: " << Stable.Region_contour.size() << std::endl;

			return Drawing;
		}

	////////////////////
		std::set<std::string> listFile(){
	        DIR *pDIR;
	        struct dirent *entry;
			std::set<std::string> files_to_read;

	        if( pDIR=opendir(base_path.c_str()) ){
				while(entry = readdir(pDIR)){
					if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 ){
						std::string const fullString = entry->d_name;

						if (fullString.length() >= gt_ending.length()) {
							if (0 == fullString.compare (fullString.length() - gt_ending.length(), gt_ending.length(), gt_ending)){
								std::string filename= fullString.substr (0,fullString.length() - gt_ending.length());
								files_to_read.insert(filename);
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


	/////////////////
		void save_decomposed_image_color(std::string path, cv::Mat image_in, std::vector <cv::Vec3b> colormap, std::map<int,int> original_map){
			double min, max;
			std::vector <cv::Vec3b> color_vector;
			cv::Vec3b black(0, 0, 0);
			color_vector.push_back(black);
			
			std::map<int,int>::iterator map_iter;
			
			cv::minMaxLoc(image_in, &min,&max);
			color_vector.resize(max);

			for(int i=1;i<= max; i++){
				map_iter = original_map.find(i);
				if (map_iter != original_map.end()){
					int index_in_original = map_iter->second;
					color_vector[i]=colormap[index_in_original];
				}
				else{		
					cv::Vec3b color(rand() % 255,rand() % 255,rand() % 255);
					color_vector[i] = color;
				}
			}
			/////
			cv::Mat image_float = cv::Mat::zeros(image_in.size(), CV_8UC3);
			for(int i=0; i < image_in.rows; i++){
				for(int j=0;j< image_in.cols; j++){
					int color_index = image_in.at<uchar>(i,j);
					image_float.at<cv::Vec3b>(i,j) = color_vector[color_index];
				}
			}
			cv::imwrite( path , image_float );



		}

	/////////////////
		std::vector <cv::Vec3b> save_image_original_color(std::string path, cv::Mat image_in){

			double min, max;			
			std::vector <cv::Vec3b> color_vector;
			cv::Vec3b black(0, 0, 0);
			color_vector.push_back(black);
			
			cv::minMaxLoc(image_in, &min,&max);

			for(int i=0;i<= max; i++){
				cv::Vec3b color(rand() % 255,rand() % 255,rand() % 255);
				color_vector.push_back(color);
			}
			cv::Mat image_float = cv::Mat::zeros(image_in.size(), CV_8UC3);
			for(int i=0; i < image_in.rows; i++){
				for(int j=0;j< image_in.cols; j++){
					int color_index = image_in.at<uchar>(i,j);
					image_float.at<cv::Vec3b>(i,j) = color_vector[color_index];
				}
			}
			cv::imwrite( path , image_float );

			return color_vector;
		}

	////////////////
		void extract_results(results& pixel, results& Regions){
			pixel.precision = 100*pixel_precision;
			pixel.recall    = 100*pixel_recall;
			
			
			float cum_precision=0;
			float cum_recall=0;
			int size_precision=0, size_recall=0;

			for(int j=0; j < Precisions.back().size();j++){
				cum_precision += Precisions.back()[j];
				size_precision++;
			}
			for(int j=0; j < Recalls.back().size();j++){
				cum_recall    += Recalls.back()[j];
				size_recall++;
			}
			Regions.precision = cum_precision/Precisions.back().size();
			Regions.recall    = cum_recall/Recalls.back().size();


//			Precisions.clear();
//			Recalls.clear();
		}

	////////////////////
		void process_all_files(){
			std::set<std::string> files_to_read = listFile();
			

			for (std::set<std::string>::iterator file_iter = files_to_read.begin() ; file_iter != files_to_read.end() ; file_iter++){
				std::cout << "Reading file  "<< *file_iter << std::endl<< std::endl<< std::endl<< std::endl;
				process_files(*file_iter);
				publish_Image();
			}
			
			float cum_precision=0, cum_quad_precision =0;
			float cum_recall=0, cum_quad_recall=0;
			int size_precision=0, size_recall=0;
			for(int i=0; i < Precisions.size();i++){
				for(int j=0; j < Precisions[i].size();j++){
					cum_precision += Precisions[i][j];
					cum_quad_precision += Precisions[i][j]*Precisions[i][j];
					size_precision++;
				}
			}			
			for(int i=0; i < Recalls.size();i++){
				for(int j=0; j < Recalls[i].size();j++){
					cum_recall    += Recalls[i][j];
					cum_quad_recall    += Recalls[i][j]*Recalls[i][j];
					size_recall++;
				}
			}
			double this_precision = cum_precision/size_precision;
			double this_recall    = cum_recall/size_recall;
			
			double this_quad_precision = cum_quad_precision/size_precision;
			double this_quad_recall    = cum_quad_recall/size_recall;

			double std_precision = sqrt(this_quad_precision - this_precision*this_precision );
			double std_recall = sqrt(this_quad_recall - this_recall*this_recall );
			
			
			
			printf(" Precision: %.1f +/- %.1f, Recall: %.1f +/- %.1f \n",this_precision, std_precision, this_recall, std_recall);









			cum_precision=0, cum_quad_precision =0;
			cum_recall=0, cum_quad_recall=0;
			for(int i=0; i < Precisions.size();i++){
				float cum_inside_precision = 0;
				for(int j=0; j < Precisions[i].size();j++){
					cum_inside_precision += Precisions[i][j];
				}
				cum_precision      += cum_inside_precision/Precisions[i].size();
				std::cout << "current precision " <<  cum_inside_precision/Precisions[i].size() << std::endl;
				cum_quad_precision += cum_inside_precision/Precisions[i].size()*cum_inside_precision/Precisions[i].size();
			}			
			size_precision=Precisions.size();

			for(int i=0; i < Recalls.size();i++){
				float cum_inside_recall = 0;
				for(int j=0; j < Recalls[i].size();j++){
					cum_inside_recall    += Recalls[i][j];
				}
				cum_recall 		+= cum_inside_recall/Recalls[i].size();
				cum_quad_recall	+= cum_inside_recall/Recalls[i].size()*cum_inside_recall/Recalls[i].size();
				std::cout << "current recall " <<  cum_inside_recall/Recalls[i].size() << std::endl;
			}
			size_recall=Recalls.size();

			this_precision = cum_precision/size_precision;
			this_recall    = cum_recall/size_recall;
			
			this_quad_precision = cum_quad_precision/size_precision;
			this_quad_recall    = cum_quad_recall/size_recall;
			
			std_precision = sqrt(this_quad_precision - this_precision*this_precision );
			std_recall = sqrt(this_quad_recall - this_recall*this_recall );
			
			
			
			printf("separated Precision: %.1f +/- %.1f, Recall: %.1f +/- %.1f \n",this_precision, std_precision, this_recall, std_recall);




			
			
		}

};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "evaluation");
	
	
	float decomp_th=2.7;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(decomp_th);
	ros::spin();
	
	return 0;
}

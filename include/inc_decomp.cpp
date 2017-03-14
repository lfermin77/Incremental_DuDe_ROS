#include "inc_decomp.hpp"



cv::Mat Stable_graph::draw_stable_contour(){
	cv::Mat Drawing = cv::Mat::zeros(image_size.height, image_size.width, CV_8UC1);	
	std::list<int> a;
	for(int i = 0; i < Region_contour.size();i++){
		drawContours(Drawing, Region_contour, i, index_to_color[i], -1, 8);
	}

	
	
	cv::flip(Drawing,Drawing,0);/*
	for(int i = 0; i < Region_centroid.size();i++){
		stringstream mix;      mix<<i;				std::string text = mix.str();
		putText(Drawing, text, cv::Point(Region_centroid[i].x, image_size.height - Region_centroid[i].y ), 
											cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Region_centroid.size()+1, 1, 8);
	}	*/
	return Drawing;
}

Stable_graph::Stable_graph(){
	max_color=0;
}

Incremental_Decomposer::Incremental_Decomposer(){
	Decomp_threshold_ = 2.5;
	resolution=0.05; //default;
	safety_distance = 0.5;//default
	first_time = true;
	current_origin_ = cv::Point2f(0,0);
}

Incremental_Decomposer::~Incremental_Decomposer(){
}

/////////////////////////////////
///// MAIN FUNCTION
Stable_graph Incremental_Decomposer::decompose_image(cv::Mat image_cleaned, float pixel_Tau, cv::Point2f origin, float resolution_in){
	
	new_origin_ = origin;// because it is first time
	resolution = resolution_in;
	
	clock_t begin_process, end_process;
	float elapsed_secs_process;
	cv::Rect resize_rect;

	if(current_origin_ != new_origin_){
		adjust_stable_contours();
	}
	
//////////////////////////////////////////////////////////
//// Decomposition
	begin_process = clock();

	cv::Mat stable_drawing = cv::Mat::zeros(image_cleaned.size().height, image_cleaned.size().width, CV_8UC1);
	drawContours(stable_drawing, Stable.Region_contour, -1, 255, -1, 8);
	
	cv::Mat working_image;
	working_image = image_cleaned & ~stable_drawing;
//		compare(image_cleaned, stable_drawing, working_image, cv::CMP_NE);
	cv::Mat will_be_destroyed = working_image.clone();
	
	std::vector<std::vector<cv::Point> > Differential_contour;
	cv::findContours(will_be_destroyed, Differential_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

		
	// multiple contours
	cout << "multiple contours "<<endl;
	
	int gap = safety_distance / resolution;
	
	vector<vector<cv::Point> > big_contours_vector;
	for(int i=0; i < Differential_contour.size(); i++){
		float current_area = cv::contourArea(Differential_contour[i]);
		if(current_area >gap*gap){
			big_contours_vector.push_back(Differential_contour[i]);
		}
	}
	if(first_time) resize_rect = cv::boundingRect(big_contours_vector[0]);
	else resize_rect= previous_rect;

	// Match between old and new
	cout << "Match between old and new "<<endl;
	vector<vector<cv::Point> > connected_contours, unconnected_contours;
	std::vector<cv::Point> unconnected_centroids, connected_centroids;
	vector< vector <int > > conection_prev_new;

//	Stable.index_to_color.clear();
	
	for(int i=0;i < Stable.Region_contour.size();i++){
		bool is_stable_connected = false;
		for(int j=0;j < big_contours_vector.size();j++){
			int connected = 0;
			cv::Point centroid;
			are_contours_connected(Stable.Region_contour[i], big_contours_vector[j] , centroid, connected);
			if (connected>0){
//							cout << "contour "<< j << " in region " <<i<< " is connected to stable region "<<k << endl;
				vector <int> pair;
				pair.push_back(i);
				pair.push_back(j);
				conection_prev_new.push_back(pair);
				is_stable_connected = true;
//						cout << "Old contour " << i<<" connected to new "<< j << endl;
			}
		}
		if(is_stable_connected == false){
			unconnected_contours.push_back(Stable.Region_contour[i]);
			unconnected_centroids.push_back(Stable.Region_centroid[i]);
//			Stable.index_to_color[unconnected_contours.size()-1]=i;
//					cout << "Old contour " << i<<" is not connected  "<< endl;
		}
		else{
			connected_contours.push_back(Stable.Region_contour[i]);
			connected_centroids.push_back(Stable.Region_centroid[i]);
		}
	}


	
	//Draw image with expanded contours matched
	cout << "Draw image with expanded contours matched "<<endl;
	cv::Mat expanded_drawing = cv::Mat::zeros(image_cleaned.size().height, image_cleaned.size().width, CV_8UC1);
	drawContours(expanded_drawing, connected_contours,  -1, 2, -1, 8);

	for(int i=0; i < conection_prev_new.size();i++){
		drawContours(expanded_drawing, big_contours_vector, conection_prev_new[i][1] , 2, -1, 8);
	}


	will_be_destroyed = expanded_drawing.clone();			
	std::vector<std::vector<cv::Point> > Expanded_contour;
	cv::findContours(will_be_destroyed, Expanded_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

	if(first_time){
		Expanded_contour.clear();
		Expanded_contour = big_contours_vector;
		first_time=false;
	}

//		cout << "Expanded_contour.size "<<Expanded_contour.size()<< endl;
	
//		end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in paint expanded diference "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;


	// Decompose in several wrappers
	cout << "Decompose in several wrappers "<<endl;
	begin_process = clock();

	vector<DuDe_OpenCV_wrapper> wrapper_vector(Expanded_contour.size());
	for(int i = 0; i <Expanded_contour.size();i++){
		cv::Mat Temporal_Image = cv::Mat::zeros(image_cleaned.size().height, image_cleaned.size().width, CV_8UC1);								
		cv::Mat temporal_image_cut = cv::Mat::zeros(image_cleaned.size().height, image_cleaned.size().width, CV_8UC1);								
		drawContours(Temporal_Image, Expanded_contour, i, 255, -1, 8);
		image_cleaned.copyTo(temporal_image_cut,Temporal_Image);
		
		wrapper_vector[i].set_pixel_Tau(pixel_Tau);			
		resize_rect |= wrapper_vector[i].Decomposer(temporal_image_cut);
	}	




// Paint differential contours
	cout << "Paint differential contours "<<endl;
	vector<vector<cv::Point> > joint_contours = unconnected_contours, new_contours;
	vector<cv::Point> joint_centroids = unconnected_centroids;
	
	for(int i = 0; i < wrapper_vector.size();i++){
		for(int j = 0; j < wrapper_vector[i].Decomposed_contours.size();j++){
//			joint_contours.push_back(wrapper_vector[i].Decomposed_contours[j]);
			new_contours.push_back(wrapper_vector[i].Decomposed_contours[j]);
			joint_centroids.push_back(wrapper_vector[i].contours_centroid[j]);
		}
	}	
//	if(new_contours.size()>0)
		cerr << "The new differential decomposition has  "<< new_contours.size() <<" contours and the old had "<< connected_contours.size() << endl;

	joint_contours.insert(joint_contours.end(), new_contours.begin(), new_contours.end() );
		cerr << "The new decomposition has  "<< joint_contours.size() <<" contours and the old had "<< Stable.Region_contour.size() << endl;
	
	
//		end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in process multiple Decomp "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;

//////////////////////////
// Order Contours
	std::map<int,vector<cv::Point> > mapped_contours;
	std::map<int,cv::Point> mapped_centroids;
	
	std::vector<int> vec_with_size_of_new(joint_contours.size(),0);// New
	std::vector<int> vec_with_size_of_old(Stable.Region_contour.size(),0);// Old
	
	std::vector<std::vector<int> > merge_matrix(Stable.Region_contour.size()  ,  vec_with_size_of_new );// Old * New
	std::vector<std::vector<int> > trans_split_matrix(Stable.Region_contour.size()  ,  vec_with_size_of_new );
	std::vector<int> merge_acum( joint_contours.size() , 0);
	
//	std::vector<std::vector<int> > split_matrix(joint_contours.size()  ,  vec_with_size_of_old );// New * Old
	std::vector<int> split_acum( Stable.Region_contour.size() , 0);			

	std::vector< std::pair<int,int> > pivots;
	std::map<int,int> new_tag_to_old_color;
		
	for(int i=0;i < Stable.Region_contour.size(); i++){
		for(int j=0; j < joint_contours.size(); j++){
			double point_inside = pointPolygonTest(joint_contours[j], Stable.Region_centroid[i] , false);
			merge_matrix[i][j] = (point_inside>=0)? 1 : 0;
			merge_acum[j]+= merge_matrix[i][j];					

			point_inside = pointPolygonTest(Stable.Region_contour[i], joint_centroids[j] , false);			
			trans_split_matrix[i][j] = (point_inside>=0)? 1 : 0;
			split_acum[i] +=  trans_split_matrix[i][j];				
			
			if(  (merge_matrix[i][j] ==1) && (trans_split_matrix[i][j]) ){
				std::pair<int,int> correspondence(i,j);
				pivots.push_back(correspondence);
				new_tag_to_old_color[j] = Stable.index_to_color[i];
				
			}
			//			
		}
	}
	


	std::cerr << "Max color was: "<< Stable.max_color << std::endl;
	
	std::cerr << "Merge Acum: ";
	for (int i=0; i < merge_acum.size(); i++)
		std::cerr << " "<< merge_acum[i];
	std::cerr << std::endl;
	

	for (int j=0; j < joint_contours.size(); j++){
		if(merge_acum[j] >1){
			std::cerr << " Region "<< j<< " comes from (possible) the merge (";			
			for (int i=0; i < Stable.Region_contour.size(); i++){		
				if(merge_matrix[i][j]>0)	
					std::cerr <<" "<< i;
			}
			std::cerr << ")"<<std::endl;
		}
		if(merge_acum[j] == 0){
			std::cerr << " Region "<< j<< " is created"<<std::endl;
			Stable.max_color ++;
//			max_color++;
			new_tag_to_old_color[j]=Stable.max_color;
//			new_tag_to_old_color[j]=max_color;
		}
	}



	std::cerr << "Split Acum: ";
	for (int i=0; i < split_acum.size(); i++)
		std::cerr << " "<< split_acum[i];
	std::cerr << std::endl;


	for (int j=0; j < Stable.Region_contour.size(); j++){
		if(split_acum[j] >1){
			std::cerr << " Regions (";			
			for (int i=0; i < joint_contours.size(); i++){		
				if(trans_split_matrix[j][i]>0)	
					std::cerr <<" "<< i;
			}
			std::cerr<< ") comes from the (possible) split of region "<< j <<std::endl;
		}
		if(split_acum[j] ==0){
			std::cerr << " Regions "<< j <<  " is destroyed"<< std::endl;
		}
	}

	std::cerr << "Pivots: ";
	for (int i=0; i < pivots.size(); i++)
		std::cerr << "("<< pivots[i].first << ","<<pivots[i].second<<") ";
	std::cerr << std::endl;


	if(new_tag_to_old_color.size() !=  joint_centroids.size()){
		for(int i=0; i < joint_centroids.size(); i++){
			std::map<int, int >::iterator index_found =  new_tag_to_old_color.find(i);
			if(index_found == new_tag_to_old_color.end()){
				Stable.max_color ++;
				new_tag_to_old_color[i]=Stable.max_color;
			}
			//	
		}
		//
	}




///////////////////
//// Build stable graph
	begin_process = clock();
	/*
	Stable.index_to_color.clear();
	for (int i=0; i < joint_contours.size(); i++)
		Stable.index_to_color[i]=i+1;
	//*/
	std::cerr << "New Max color is: "<< Stable.max_color << std::endl;
	
	Stable.index_to_color = new_tag_to_old_color;
//*
	Stable.Region_contour  = joint_contours;
	Stable.Region_centroid = joint_centroids;
	
	Stable.diagonal_centroid.clear();
	Stable.diagonal_connections.clear();
	for(int i=0; i < joint_centroids.size(); i++){	
		for(int j=0; j < i; j++){
			int connected;
			cv::Point centroid;
//			are_contours_connected(vector<cv::Point> first_contour, vector<cv::Point> second_contour, cv::Point &centroid, int &number_of_ones );
			are_contours_connected(Stable.Region_contour[i], Stable.Region_contour[j], centroid, connected );
			if (connected>0){
//							cout << "contour "<< j << " in region " <<i<< " is connected to stable region "<<k << endl;
				set <int> pair;
				pair.insert(i);
				pair.insert(j);

				Stable.diagonal_centroid.push_back(centroid);
				Stable.diagonal_connections.push_back(pair);

						cerr << "Old contour " << i<<" connected to new "<< j << endl;
			}
//			frontiers_in_map();
			
		}
	}	
	
	
	//*/
	/*
	Stable.Region_contour  = ordered_contours;
	Stable.Region_centroid = ordered_centroids;
	//*/
	previous_rect = resize_rect;
	
	Stable.image_size = image_cleaned.size();

//		end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in process Stable Graph "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;

	
		
	return Stable;


}


/////////////////////////////
//// UTILITIES

void Incremental_Decomposer::are_contours_connected(vector<cv::Point> first_contour, vector<cv::Point> second_contour, cv::Point &centroid, int &number_of_ones ){
	
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
	if(number_of_ones>0){
		centroid.x = acum.x/number_of_ones;
		centroid.y = acum.y/number_of_ones;
	}
}
	
void Incremental_Decomposer::adjust_stable_contours(){
	int a;
	cout<<"Adjusting Contours "<< endl << endl;

	cv::Point correction;
	
// considering constant resolution
	correction.x = (current_origin_.x - new_origin_.x) / resolution;
	correction.y = (current_origin_.y - new_origin_.y) / resolution;
	
	for(int i=0; i < Stable.Region_contour.size();i++){
		Stable.Region_centroid[i] += correction;
		for(int j=0; j < Stable.Region_contour[i].size();j++){
			Stable.Region_contour[i][j] += correction;
//					Stable.Region_contour[i][j] = cartesian_to_pixel(pixel_to_cartesian(Stable.Region_contour[i][j]));
		}
	}
		
	current_origin_ = new_origin_;
	
	
}

cv::Point Incremental_Decomposer::pixel_to_cartesian(cv::Point point_in){
	cv::Point point_out;
	point_out.x = point_in.x * resolution + current_origin_.x;
	point_out.y = point_in.y * resolution + current_origin_.y;
	
	return point_out;
}

cv::Point Incremental_Decomposer::cartesian_to_pixel(cv::Point point_in){
	cv::Point point_out;
	point_out.x = (point_in.x - new_origin_.x) / resolution ;
	point_out.y = (point_in.y - new_origin_.y) / resolution ;			
	
	return point_out;
}


void Incremental_Decomposer::frontiers_in_map(cv::Mat  Tag_image, cv::Mat  original_image){
	
	int window_size=1;
	
	std::map < std::set<int> , std::vector<cv::Point>   > points_in_edge, frontier_points;
	std::map < int , std::vector<cv::Point>   > region_contour;
	
	for (int i=window_size;i < Tag_image.size().width- window_size ;i++){
		for (int j=window_size;j < Tag_image.size().height - window_size ;j++){
		
			/////////////////////
			cv::Point window_center(i,j);
			int center_tag = Tag_image.at<uchar>(window_center);
			
			std::set<int>  connections_in_region, frontier_connections;
			//check neigbourhood for frontiers and connection
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
					if ( frontier==255 &&  tag==0) frontier_connections.insert( -1 );

				}
			}
			 
			//////////////////////////////
			if (connections_in_region.size()==2 || (frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ) ){
				region_contour[center_tag-1].push_back(window_center);
			}

			//////////////////
			if(connections_in_region.size()==2){					
				points_in_edge[connections_in_region].push_back(window_center);
				
			}
			if(frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ){					
				frontier_points[frontier_connections].push_back(window_center);
			}
			
		}
	}
	//
	std::cerr << "points_in_edge "<< points_in_edge.size() << std::endl;

	Stable.diagonal_connections.clear();
	Stable.diagonal_centroid.clear();

	for(std::map < std::set<int> , std::vector<cv::Point>   > ::iterator map_iter =points_in_edge.begin(); map_iter !=points_in_edge.end(); map_iter ++){
		std::set<int> current_edge = map_iter->first;
		cv::Point centroid_acum(0,0);
		std::vector<cv::Point> current_contour = map_iter->second;
		for (int i=0; i <current_contour.size();i++){
			centroid_acum += current_contour[i];
		}
		centroid_acum.x= centroid_acum.x/current_contour.size();
		centroid_acum.y= centroid_acum.y/current_contour.size();
		
		Stable.diagonal_connections.push_back(current_edge);
		Stable.diagonal_centroid.push_back(centroid_acum);
		
	}


	std::cerr << "frontier_points "<< frontier_points.size() << std::endl;
	std::cerr << "region_contour "<< region_contour.size() << std::endl;

}

cv::Point contour_centroid(std::map < std::set<int> , std::vector<cv::Point>   >  input_map){
	cv::Point centroid(0,0);
	
//	for(std::map < std::set<int> , std::vector<cv::Point>   > ::iterator map_iter =input_map.begin(); map_iter !=input_map.end(); map_iter ++){
		


	return centroid;
}


std::vector < std::vector<cv::Point> > Incremental_Decomposer::decompose_edge(	 std::vector<cv::Point>   points_in_edge){
	std::vector < std::vector<cv::Point> > contours_divided;
	
	cv::Rect rectangle = boundingRect(points_in_edge);
	
	
	
	
	return contours_divided;
}



























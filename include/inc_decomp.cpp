#include "inc_decomp.hpp"



cv::Mat Stable_graph::draw_stable_contour(){
	cv::Mat Drawing = cv::Mat::zeros(image_size.height, image_size.width, CV_8UC1);	
	std::list<int> a;
	for(int i = 0; i < Region_contour.size();i++){
		drawContours(Drawing, Region_contour, i, i+1, -1, 8);
	}


	
	
	
	cv::flip(Drawing,Drawing,0);/*
	for(int i = 0; i < Region_centroid.size();i++){
		stringstream mix;      mix<<i;				std::string text = mix.str();
		putText(Drawing, text, cv::Point(Region_centroid[i].x, image_size.height - Region_centroid[i].y ), 
											cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Region_centroid.size()+1, 1, 8);
	}	*/
	return Drawing;
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
	Stable.map_old_tag_to_new.clear();
	
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
			Stable.map_old_tag_to_new[unconnected_contours.size()-1]=i;
//					cout << "Old contour " << i<<" is not connected  "<< endl;
		}
		else{
			connected_contours.push_back(Stable.Region_contour[i]);
			connected_centroids.push_back(Stable.Region_centroid[i]);
		}
	}

/*
	cout << "number of growing regions " << conection_prev_new.size() << endl;
	cout << "connected_contours.size " << connected_contours.size() << endl;
	cout << "unconnected_contours.size " << unconnected_contours.size() << endl;
	cout << "Sum " << connected_contours.size() + unconnected_contours.size() << endl;
	cout << "Original " << Stable.Region_contour.size() << endl;
//*/			

	
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
	std::vector<int> merge_acum( joint_contours.size() , 0);
	
	std::vector<std::vector<int> > split_matrix(joint_contours.size()  ,  vec_with_size_of_old );// New * Old
	std::vector<int> split_acum( Stable.Region_contour.size() , 0);			
	
	for(int i=0;i < Stable.Region_contour.size(); i++){
		for(int j=0; j < joint_contours.size(); j++){
			double point_inside = pointPolygonTest(joint_contours[j], Stable.Region_centroid[i] , false);
			merge_matrix[i][j] = (point_inside>=0)? 1:0;
			merge_acum[j]+= merge_matrix[i][j];		
			
			//*
			point_inside = pointPolygonTest(Stable.Region_contour[i], joint_centroids[j] , false);
			split_matrix[j][i] = (point_inside>=0)? 1:0;
			split_acum[i] +=  split_matrix[j][i];	
			//*/
		}
	}
	
	/*

	for(int i=0;i < joint_contours.size(); i++){
		for(int j=0; j < Stable.Region_contour.size(); j++){
			double point_inside = pointPolygonTest(Stable.Region_contour[j], joint_centroids[i] , false);
			split_matrix[i][j] = (point_inside>=0)? 1:0;
			split_acum[j] +=  split_matrix[i][j];
		}
	}
	//*/
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
	}



	std::cerr << "Split Acum: ";
	for (int i=0; i < split_acum.size(); i++)
		std::cerr << " "<< split_acum[i];
	std::cerr << std::endl;

	for (int j=0; j < Stable.Region_contour.size(); j++){
		if(split_acum[j] >1){
			std::cerr << " Regions (";			
			for (int i=0; i < joint_contours.size(); i++){		
				if(split_matrix[i][j]>0)	
					std::cerr <<" "<< i;
			}
			std::cerr<< ") comes from the (possible) split of region "<< j <<std::endl;
		}
	}






	

	std::list<int> remaining_contours;
	
	for(int i=0; i < joint_centroids.size();i++)
		remaining_contours.push_back(i);


		cerr << "remaining_contours FIRST TIME  "<< remaining_contours.size() << endl;
	
	for(int i=0; i <Stable.Region_contour.size();i++){
		cv::Point current_centroid = Stable.Region_centroid[i];
		float min_dist = std::numeric_limits<float>::infinity();
		int min_index=-1;

		//*

		cerr << "remaining_contours.size()  "<< remaining_contours.size() << endl;

		double point_inside = -1;

		for(std::list<int>::iterator list_iter = remaining_contours.begin(); list_iter != remaining_contours.end(); list_iter++){
			point_inside = pointPolygonTest(joint_contours[*list_iter], current_centroid, false);
			if(point_inside >0 ){
				cerr << "Centroid of contour  "<< i << " is inside " <<  *list_iter << endl;
				min_index=*list_iter;
			}

		}
		if(min_index >= 0){
			mapped_contours [i] = (joint_contours [min_index]);
			mapped_centroids[i] = (joint_centroids[min_index]);
			remaining_contours.remove(min_index);			

			
		}
		else{
				cerr << "PREVIOUS DISAPPEARED" << endl;
		}
		//*/


		/*
		for(std::list<int>::iterator list_iter = remaining_contours.begin(); list_iter != remaining_contours.end(); list_iter++){ //Ordering with centroid distances
			float diff = cv::norm( joint_centroids[*list_iter] - current_centroid  );
			if(diff < min_dist){
				min_index=*list_iter;
				min_dist = diff;
			}
		}
		if(min_index>=0){
			ordered_contours .push_back(joint_contours [min_index]);
			ordered_centroids.push_back(joint_centroids[min_index]);
			remaining_contours.remove(min_index);
		}
		//*/
	}
	/*
	if(remaining_contours.size()>0){
		cerr << "Contours new  "<<   endl;
		for(std::list<int>::iterator list_iter = remaining_contours.begin(); list_iter != remaining_contours.end(); list_iter++){
				ordered_contours .push_back(joint_contours [*list_iter]);
				ordered_centroids.push_back(joint_centroids[*list_iter]);
		}
	}
	//*/
	
	vector<vector<cv::Point> > ordered_contours;
	vector<cv::Point> ordered_centroids;
	std::list<int>::iterator list_iter = remaining_contours.begin();

	for(int i=0; i < joint_centroids.size();i++){
		std::map<int,vector<cv::Point> >::iterator index_found = mapped_contours.find(i);
		std::map<int,cv::Point>::iterator centroid_index = mapped_centroids.find(i);
		
		if(index_found != mapped_contours.end()){
			ordered_contours.push_back(index_found->second);
			ordered_centroids.push_back(centroid_index->second);

		}
		else{
			ordered_contours .push_back(joint_contours [*list_iter]);
			ordered_centroids.push_back(joint_centroids[*list_iter]);
			list_iter++;
		}
		//
	}
		
		
	


	

	cerr << "The new decomposition has  "<< mapped_contours.size()  << endl;
	
				cerr << endl << endl;


///////////////////
//// Build stable graph
	begin_process = clock();
/*
	Stable.Region_contour  = joint_contours;
	Stable.Region_centroid = joint_centroids;
	//*/
	//*
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





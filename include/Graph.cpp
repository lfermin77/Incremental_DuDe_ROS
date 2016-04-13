#include "Graph.hpp"



void Graph_Search::initialize_Graph(int size){
	Graph_Node_List_.resize(size);
	for (int i=0;i<Graph_Node_List_.size();i++) {
		Graph_Node_List_[i].Node_Name_ = i;
		Graph_Node_List_[i].distance_from_start_ =  std::numeric_limits<double>::infinity();//10000;
		Graph_Node_List_[i].predecesor_ = -1;
		
	}
}

void Graph_Search::initialize_Frontier(int size){
	Frontier_Node_List_.resize(size);
	for (int i=0;i<Frontier_Node_List_.size();i++) {
		Frontier_Node_List_[i].Node_Name_ = i;
		Frontier_Node_List_[i].distance_from_start_ = std::numeric_limits<double>::infinity();// 10000;
		Frontier_Node_List_[i].predecesor_ = -1;
		
	}
}

void Graph_Search::print_graph_attributes(){
	std::cout<<std::endl<<"Graph Atributes"<<std::endl<<std::endl;
	for(int i=0;i< Graph_Node_List_.size();i++){
		Graph_Node_List_[i].print_atributes();
	}
}

void Graph_Search::print_frontier_attributes(){
	std::cout<<std::endl<<"Frontier Atributes"<<std::endl<<std::endl;
	for(int i=0;i< frontier_connected_.size();i++){
		Frontier_Node_List_[frontier_connected_[i] ].print_atributes();
	}
}

void Graph_Search::print_all_frontier_attributes(){
	std::cout<<std::endl<<"Frontier Atributes"<<std::endl<<std::endl;
	for(int i=0;i< Frontier_Node_List_.size();i++){
		Frontier_Node_List_[i].print_atributes();
	}
}

void Graph_Search::print_frontier_connected(){
	std::cout<<std::endl<<"Frontiers Connected"<< std::endl;
	for(int i=0;i< frontier_connected_.size();i++){
		std::cout<< "   "<< frontier_connected_[i]<< std::endl;
	}
}


void Graph_Search::insert_edges(std::set<int> conection, float distance, int index_in_edge){
	std::set<int>::iterator it=conection.begin();
	// It only have two elements so the lazy way

	int first=*it;
	it++;
	int last=*it;

//		std::cout<<"inside func first "<< first<<" and second "<< last<< std::endl;

	if( first >= 0){
		Graph_Node_List_[ first ].connections_.push_back( last );
		Graph_Node_List_[ first ].index_in_edges_.push_back(index_in_edge);
		Graph_Node_List_[ first ].distance_to_connections_.push_back(distance/2);
	}		
	else{
		Frontier_Node_List_[ -(first)-1 ].connections_.push_back( last );
		Frontier_Node_List_[ -(first)-1 ].index_in_edges_.push_back(index_in_edge);
		Frontier_Node_List_[ -(first)-1 ].distance_to_connections_.push_back(distance/2);
	}

	
	if(last >= 0){
		Graph_Node_List_[ last ].connections_.push_back( first );
		Graph_Node_List_[ last ].index_in_edges_.push_back(index_in_edge);
		Graph_Node_List_[ last ].distance_to_connections_.push_back(distance/2);
	}		
	else{
		Frontier_Node_List_[ -(last)-1 ].connections_.push_back( first);
		Frontier_Node_List_[ -(last)-1 ].index_in_edges_.push_back(index_in_edge);
		Frontier_Node_List_[ -(last)-1 ].distance_to_connections_.push_back(distance/2);
	}
	
	
	
}


void Graph_Search::dijkstra_min_dist(){
	std::vector<int> nodes_to_explore;
	for(int i=0;i<Graph_Node_List_.size();i++)
		nodes_to_explore.push_back(i);
	
	while(nodes_to_explore.size()){
		///Find node with minimum distance
		float min_dist =std::numeric_limits<double>::infinity();
		int min_index=0;
		std::vector<int>::iterator iter_min=nodes_to_explore.begin();

		for(std::vector<int>::iterator it=nodes_to_explore.begin(); it != nodes_to_explore.end();++it){
			if( Graph_Node_List_[*it].distance_from_start_  < min_dist){
				min_dist = Graph_Node_List_[*it].distance_from_start_;
				iter_min = it;
			}
		}
		
		for(int j=0; j < Graph_Node_List_[ *iter_min ].connections_.size();j++){
			float new_distance, old_distance;	
			int current_node = Graph_Node_List_[ *iter_min ].connections_[j];
			new_distance = Graph_Node_List_[ *iter_min ].distance_from_start_ + Graph_Node_List_[ *iter_min ].distance_to_connections_[j];
			
			if(current_node >=0){
				old_distance = Graph_Node_List_[current_node].distance_from_start_;		
				if( (new_distance < old_distance)  ){
					Graph_Node_List_[current_node].distance_from_start_  = new_distance;
					Graph_Node_List_[current_node].predecesor_ = *iter_min;
				}
			}
			else{
				Frontier_Node_List_[-current_node-1].distance_from_start_  = new_distance;
				Frontier_Node_List_[-current_node-1].predecesor_ = *iter_min;
			}

		}
		nodes_to_explore.erase(iter_min);
	}

//		Graph_Search::frontiers_minima();	//chapuza
}


// Run after dijkstra
void Graph_Search::frontiers_minima(){
	std::vector<float> distances;		
	distances.resize(frontier_connected_.size() );

	std::vector<int> connected_to;		
	connected_to.resize(frontier_connected_.size() );

	float min_expected = std::numeric_limits<double>::infinity();
	int index_expected=0;

	if(frontier_connected_.size()!=1){
		for(int i=0; i< frontier_connected_.size();i++){
			distances[i] =  std::numeric_limits<double>::infinity();
			for(int j=0; j< frontier_connected_.size();j++){
				
				if(i != j){
					float distance_i_j = norm (  Frontier_Node_List_[ frontier_connected_[i] ].Node_Position_   -   Frontier_Node_List_[ frontier_connected_[j] ].Node_Position_ );
//			std::cout<<"Distance between "<< frontier_connected_[i] <<" and "<<frontier_connected_[j]<< " is "<< distance_i_j <<std::endl;
					if(distance_i_j < distances[i]){
						distances[i] = distance_i_j;
						connected_to[i] =  frontier_connected_[j] ;
					}
				}
			}
		}
		

		
		
		///Frontier to be explored

		for(int i=0; i< frontier_connected_.size();i++){
			float expected_distance = Frontier_Node_List_[ frontier_connected_[i] ].distance_from_start_    +  distances[i];
//			std::cout<<"Node "<< frontier_connected_[i] <<" with expected distance "<<expected_distance<<std::endl;
			if(expected_distance < min_expected ){
				min_expected = expected_distance;
				index_expected = i;
			}
		}

	}
		
	std::cout<<"The frontier that should be explored is "<< frontier_connected_[index_expected]<<" in node "<<	Frontier_Node_List_[frontier_connected_[index_expected]].connections_[0]
	<<" expecting connect it to frontier "<<  connected_to[index_expected] <<" in node "<<  Frontier_Node_List_[connected_to[index_expected]].connections_[0] <<std::endl;				
	
//	Frontier_Node_List_[frontier_connected_[index_expected]].print_atributes();
//	Frontier_Node_List_[connected_to[index_expected]].print_atributes();

	std::vector<int>::iterator it;
	std::vector<cv::Point>::iterator it_position;
		
//		std::cout<<"The path is "<< std::endl << "  f "<< frontier_connected_[index_expected] << std::endl;		
	node_path.push_back(-frontier_connected_[index_expected]-1);
	Positions_Path.push_back(( Frontier_Node_List_[ frontier_connected_[index_expected] ]. Node_Position_ ));
	
	

	int last_node = Frontier_Node_List_[frontier_connected_[index_expected]].predecesor_;
//		std::cout<<"  g "<< last_node << std::endl;
	
	it=node_path.begin();
	node_path.insert(it, last_node);

	it_position = Positions_Path.begin();
	Positions_Path.insert(it_position, ( Graph_Node_List_[last_node].Node_Position_));
	
	
	while(Graph_Node_List_[last_node].predecesor_ >=0  ){
		last_node = Graph_Node_List_[last_node].predecesor_;	
//			std::cout<<"  g "<< last_node << std::endl;
		it=node_path.begin();
		node_path.insert(it, last_node);		

		it_position = Positions_Path.begin();
	Positions_Path.insert(it_position, ( Graph_Node_List_[last_node].Node_Position_));
	}
	
	std::cout<<"Path in vector "<< std::endl;	
	for(int i=0; i< node_path.size();i++){
		std::cout<<"   "<< node_path[i] << "  located in "<< Positions_Path[i] <<std::endl;
	}		
	

	for(int i=0; i< (node_path.size()-1);i++){
//			std::cout<<"node in path   "<< node_path[i] <<" connected to" << std::endl;	
		for(int j=0; j < Graph_Node_List_[node_path[i]].connections_.size();j++){
//			std::cout<<"   "<< Graph_Node_List_[node_path[i]].connections_[j] << std::endl;
			
			if( Graph_Node_List_[node_path[i]].connections_[j] ==  node_path[i+1]){
				edges_path.push_back( Graph_Node_List_[node_path[i] ].index_in_edges_[j] );
			}
		}
	}




}


cv::Point Graph_Search::euclidean_closest_frontier(cv::Point robot_position){
//		std::cout<<std::endl<<"Closest Euclidean Frontier"<<std::endl<<std::endl;
	float min_dist = std::numeric_limits<float>::infinity();;
	int min_index=0;
	for(int i=0;i< Frontier_Node_List_.size();i++){
		float distance_2_robot = norm (  robot_position   -   Frontier_Node_List_[ i ].Node_Position_ );
//		std::cout << "distance_2_robot "<< distance_2_robot<< std::endl;
		if(distance_2_robot < min_dist){
			min_dist = distance_2_robot;
			min_index = i;
		}
	}
//		std::cout << "      min_dist "<< min_dist<< std::endl;
	return Frontier_Node_List_[ min_index ].Node_Position_;
}


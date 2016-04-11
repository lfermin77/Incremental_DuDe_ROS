
//openCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <limits>









class Node_to_Search{
	
	public:
	
	int Node_Name_;
	cv::Point Node_Position_;
	float distance_from_start_;
	std::vector<int> connections_;
	std::vector<float> distance_to_connections_;
	std::vector<int> index_in_edges_;
	int predecesor_;
	

	Node_to_Search(){
	}
	
	~Node_to_Search(){
	}
	
	void print_atributes(){
		std::cout <<"The node "<< Node_Name_ <<" is in "<< Node_Position_ <<" at "<< distance_from_start_<<" from start, and is connected to"<<std::endl;
		for (int i=0; i <connections_.size();i++){
			std::cout <<"   "<< connections_[i]<<" at distance "<< distance_to_connections_[i]<<std::endl;
		}
		std::cout <<" its predecesor is node "<< predecesor_<< std::endl;

	}




};


class  Graph_Search{
	public:
	std::vector<Node_to_Search> Graph_Node_List_;
	std::vector<Node_to_Search> Frontier_Node_List_;
	int starting_node_;
	std::vector<int> frontier_connected_;
	
	std::vector<int> node_path;	
	std::vector<int> edges_path;
	std::vector<cv::Point> Positions_Path;

	Graph_Search(){
	}
	
	~Graph_Search(){
	}


	void initialize_Graph(int size);


	void initialize_Frontier(int size);
	
	void print_graph_attributes();

	void print_frontier_attributes();

	void print_all_frontier_attributes();
	
	void print_frontier_connected();
	
	void insert_edges(std::set<int> conection, float distance, int index_in_edge);

	void dijkstra_min_dist();

// Run after dijkstra
	void frontiers_minima();
	
	cv::Point euclidean_closest_frontier(cv::Point robot_position);



};



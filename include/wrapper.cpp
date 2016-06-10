#include "wrapper.hpp"



///////////////////////////////////
void DuDe_OpenCV_wrapper::insert_contour_to_poly(std::vector<cv::Point> contour_in, 	c_ply& polygon ){

	polygon.beginPoly();
	for(int i=1;i <= contour_in.size();i++){
		float x = contour_in[contour_in.size()-i].x;
		float y = contour_in[contour_in.size()-i].y;
		polygon.addVertex(x, y);
	}
	polygon.endPoly();
	
}	


////////////////////////////////////////
cv::Rect DuDe_OpenCV_wrapper::Decomposer(cv::Mat Occ_Image){
	/*
///////////////////////////////	
//Occupancy Image to Free Space	
	std::cout << "Cleaning Image..... "; 		double start_cleaning = getTime();
	cv::Mat open_space = Occ_Image<10;
	cv::Mat black_image = Occ_Image>90 & Occ_Image<=100;		
	cv::Mat Median_Image, Image_in, cut_image ;
	{
		cout << "Entering........ ";
		cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			
		cout << "dilated........ ";
		cv::medianBlur(open_space, Median_Image, 3);
		cout << "Median Blur........ ";
		Image_in = Median_Image & ~black_image;
		cout << "And........ ";
		Image_in.copyTo(cut_image);			
		cout << "copy........ ";
	}
	double end_cleaning = getTime();  cout << "done, it last "<<(end_cleaning-start_cleaning)<< " ms"  << endl;			
*/
///////////////////////////////	
//// Look for the big contours
	std::cout << "Finding Contours....... "; double start_finding = getTime();
	std::vector<std::vector<cv::Point> > Explored_contour;
	std::vector<cv::Vec4i> hierarchy; //[Next, Previous, First_Child, Parent]
	cv::findContours(Occ_Image, Explored_contour, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );
		
	int current_index=0;
	cv::Rect resize_rect = boundingRect(Explored_contour[0]);
	float Big_Contour_area = cv::contourArea(Explored_contour[0]);
	int Big_Contour_Index=0;
	while(hierarchy[current_index][0] != -1){ // Find Bigger Area
		current_index=hierarchy[current_index][0];
		float current_area = cv::contourArea(Explored_contour[current_index]);
		resize_rect = resize_rect | boundingRect(Explored_contour[current_index]);
		if(current_area > Big_Contour_area){
			Big_Contour_area = current_area;
			Big_Contour_Index = current_index;
		}
	}
	double end_finding = getTime();  cout << "done, it last "<<(end_finding-start_finding)<< " ms"  << endl;			
	Parent_contour = Explored_contour [Big_Contour_Index ];
/////////////////////////////
//// Insert contours in Dual-Space Decomposer 
	std::cout << "Inserting contours ....... "; double start_inserting = getTime();
	
	c_polygon polygons;
	{
		c_ply poly_out(c_ply::POUT);
		insert_contour_to_poly( Explored_contour [Big_Contour_Index ], poly_out);
		polygons.push_back(poly_out);

		int current_child = hierarchy[ Big_Contour_Index ] [2];				
		while(current_child !=-1){ //if child, insert
			float Area = cv::contourArea(Explored_contour[current_child]);
			if(Area > Area_threshold){
				c_ply poly_in(c_ply::PIN);
				insert_contour_to_poly( Explored_contour [current_child ], poly_in);
				polygons.push_back(poly_in);
			}
			current_child = hierarchy[ current_child ] [0];
		}
	}
	double end_inserting = getTime();  cout << "done, it last "<<(end_inserting-start_inserting)<< " ms"  << endl;			
	
////////////////////////////////
//// Decompose
	//		cout << "Polygons size " << polygons.size() << endl;
	cout << "- Dual-space Decomposition with tau=" << g_tau << endl;
	float r = polygons.front().getRadius();

	if(tolerance_in_pixels){
		 g_tau = g_tau_pixel;
	 }
	 else{
		 g_tau*=r;// Scale the concavity variable if tau is fractional
	 }
	double start_dude = getTime();
	///////////////
	Dual_decompose(polygons);
	//////////////
	double end_dude = getTime();  cout << "done, decomposing "<< finalPolygonPieces.size() <<" polygons last "<<(end_dude-start_dude)<< " ms"<<endl;

////////////////////////
//Export files
//	export_all_svg_files();

///////////////////////
// Polygon to Contour
	extract_contour_from_polygon();		

//////////////////////
// Extract graph and print it
//	extract_graph();
//	print_graph();





	return resize_rect;
}

//////////////////////////////////////////////////////////////
void DuDe_OpenCV_wrapper::Dual_decompose(c_polygon& polygons){
	c_polygon* p1 = new c_polygon();	
	p1->copy(polygons);
	dude.build(*p1, g_tau, true);
	getP()=polygons;

	prepare_skeleton();
	
	draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), dude.m_PMs.begin(), dude.m_PMs.end());
	draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), dude.hole_PMs.begin(), dude.hole_PMs.end());
	updateMINMAXConcavity(dude);
	
	//debug
	polygons.build_all();

	iterativeDecompose(*p1, dude.m_cuts, g_tau, finalPolygonPieces, draw_decoration.allAccumulatedCuts, draw_decoration.se, true);
//	decomposeMoreTimes(polygons, dude.m_cuts, g_tau, finalPolygonPieces, draw_decoration.allAccumulatedCuts, draw_decoration.se, true, 0);
//	cout << "finalPolygonPieces.size() " << finalPolygonPieces.size() <<endl;		

	estimate_COM_R_Box(); //compute the bounding box of all geometries

}

//////////////////////
void DuDe_OpenCV_wrapper::prepare_skeleton()
{
	//prepare for extract skeletons
	ExtracSkeleton * m_ES = ES_Factory::create_ES("PA");
	draw_decoration.se.setExtracSkeleton(m_ES);
	draw_decoration.se.setQualityMeasure(NULL);
	draw_decoration.se.begin();
}

//////////////////////////////
void DuDe_OpenCV_wrapper::export_all_svg_files()
{
	string basename = "Decomposed";
	int psize = finalPolygonPieces.size();

	c_polygon & P = getP();
	double * bbox = P.getBBox();
	double width = bbox[1] - bbox[0];
	double height = bbox[3] - bbox[2];
	double stroke_width = width / 200;
	svg::Dimensions dimensions(800, 800);
	svg::Dimensions svg_viewbox_dim(width + stroke_width * 2, height + stroke_width * 2);
	svg::Point svg_viewbox_org(bbox[0] - stroke_width, height - bbox[3] + stroke_width);

	stringstream ss;
	ss << basename.c_str() << "_dude2d.svg";
	string sname = ss.str();
	svg::Document doc(sname, svg::Layout(dimensions, svg::Layout::BottomLeft, svg_viewbox_dim, svg_viewbox_org));


	for (int i = 0; i < psize; i++)
	{
		svg::Color randColor(svg::Color::Defaults(svg::Color::Aqua + (i % 15)));
		svg::Polygon p_i(svg::Fill(randColor), svg::Stroke(stroke_width, svg::Color::Black));
		finalPolygonPieces[i]->toSVG(p_i);
		doc << p_i;
	}

	doc.save();
	cout << "- Saved: " << sname << endl;
}

/////////////////////////////////////
void DuDe_OpenCV_wrapper::estimate_COM_R_Box() 
{
	getP().buildBoxAndCenter();
	getQ().buildBoxAndCenter();

	for (short i = 0; i < 4; i++)
		draw_decoration.box[i] = getP().getBBox()[i]; //+getQ().getBBox()[i];

	draw_decoration.COM.set((draw_decoration.box[1] + draw_decoration.box[0]) / 2, (draw_decoration.box[3] + draw_decoration.box[2]) / 2);
	draw_decoration.R = sqr(draw_decoration.COM[0]-draw_decoration.box[0]) + sqr(draw_decoration.COM[1] - draw_decoration.box[2]);
	draw_decoration.R = sqrt((float) draw_decoration.R); //*0.95;

	//rebuild box
	draw_decoration.box[0] = draw_decoration.COM[0] - draw_decoration.R;
	draw_decoration.box[1] = draw_decoration.COM[0] + draw_decoration.R;
	draw_decoration.box[2] = draw_decoration.COM[1] - draw_decoration.R;
	draw_decoration.box[3] = draw_decoration.COM[1] + draw_decoration.R;

	//estimate normal length based on R
	draw_decoration.normal_length = draw_decoration.R / 20;
}

/////////////////////////////
void DuDe_OpenCV_wrapper::extract_contour_from_polygon(){

	Decomposed_contours.clear();
	for(int i=0; i < finalPolygonPieces.size(); i++){
		c_polygon temp_polygon = *finalPolygonPieces[i];
		list<c_ply>::iterator temp_iter = temp_polygon.begin();
		
		std::vector<cv::Point> current_contour;

		ply_vertex * ptr = temp_iter->getHead(); // There's only one polygon after decomposed
		do{
			const Point2d& pos = ptr->getPos();
			cv::Point point_to_add(pos[0], pos[1]);
			current_contour.push_back(point_to_add);
			ptr = ptr->getNext();
		} 
		while (ptr!=temp_iter->getHead());
		Decomposed_contours.push_back(current_contour);

	}
	// cout << "Decomposed_contours.size() " << Decomposed_contours.size() <<endl;
	for(int i=0;i<Decomposed_contours.size();i++){
		cv::Moments moments = cv::moments(Decomposed_contours[i], true);
		cv::Point center = cv::Point( moments.m10/moments.m00 , moments.m01/moments.m00 );
		contours_centroid.push_back(center);
	}
}

///////////////////////////////////////
void DuDe_OpenCV_wrapper::extract_graph(){
	//////////////Diagonals are the edges and contours are the nodes
	vector<c_diagonal> Diagonals = dude.getFinalCuts();		
	contours_connections.resize(Decomposed_contours.size());

	////// Find diagonals in contours	
	int diag_count=0;
	diagonal_centroid.clear();
	diagonal_connections.clear();
	
	for (vector<c_diagonal>::iterator Diag_it = Diagonals.begin(); Diag_it != Diagonals.end(); ++Diag_it){	
		std::set<int> contours_connected;
		ply_vertex * vertex1 = Diag_it->getV1();
		const Point2d& pos_vertex1 = vertex1->getPos();
		ply_vertex * vertex2 = Diag_it->getV2();
		const Point2d& pos_vertex2 = vertex2->getPos();

		cv::Point Diag_avg = cv::Point( (pos_vertex1[0] + pos_vertex2[0])/2, (pos_vertex1[1] + pos_vertex2[1])/2  );
		diagonal_centroid.push_back(Diag_avg);

		for(int contour_count = 0; contour_count < Decomposed_contours.size(); contour_count++){
			std::vector<cv::Point> current_contour = Decomposed_contours[contour_count];

			if( abs( cv::pointPolygonTest(current_contour, Diag_avg, true) ) <= 2){
				contours_connected.insert(contour_count);
			}
		}
		diagonal_connections.push_back(contours_connected);
		
		for(std::set<int>::iterator  set_it = contours_connected.begin();set_it!=contours_connected.end();set_it++){
			int connected_from = *set_it;
			for(std::set<int>::iterator  set_it2 = contours_connected.begin();set_it2!=contours_connected.end();set_it2++){
				int connected_to = *set_it2;
				if(connected_from != connected_to){
					contours_connections[connected_from].insert(connected_to);
				}
			}
		}
	// */
	}
}

////////////////////////////	
void DuDe_OpenCV_wrapper::print_graph(){
	std::cout <<"First, Nodes"<< std::endl;
	for(int i=0;i<contours_centroid.size();i++){
		std::cout << " "<< i;
//			std::cout <<" located in "<< contours_centroid[i];
		//*
		std::cout << "  connected to (" ;
		for(std::set<int>::iterator  set_it = contours_connections[i].begin();set_it!=contours_connections[i].end();set_it++){
			int connected_from = *set_it;
			std::cout << " "<< connected_from;
		}
		// */
		
		std::cout << " )"<< std::endl;
	}
	std::cout << std::endl;
}


///////////////////////////
void DuDe_OpenCV_wrapper::measure_performance(){
	vector<float> convexities, compactness, qualities, areas;
	float Total_Area=0;
	float Total_Quality=0;

	// Calculate Quality
		// Region Quality
			// Convexity with area/area_convex_hull (c_i)
			// Compactness second moment (s_i) =(1/(Mi*Ai))* \sum(((xi-x0)^2 + (yi-y0)^2 )/(Ai))
			//  q_i = c_i -s_i
	
	for (int i=0; i < Decomposed_contours.size(); i++){

		cv::Moments moments = cv::moments(Decomposed_contours[i], true);
		vector<cv::Point> hull;
		
		convexHull( Decomposed_contours[i], hull );	
		float A_i=moments.m00;
		float H_i = cv::contourArea(hull);
		float c_i = A_i/H_i;
		convexities.push_back( c_i);
		areas.push_back( A_i);
		float M_i=A_i;// currently the number of cells is the number of pixels

		float s_i = moments.mu20 + moments.mu02;
		s_i = (1/(M_i*A_i))*(s_i/A_i);
		compactness.push_back(s_i);
		
//		cout << "convexity "<< convexities.back() <<", area " << areas.back() << ", compactness "<< s_i<<endl;	
		float q_i = c_i - s_i;
		qualities.push_back(q_i);
		cout << "quality " << q_i << endl;
		
		Total_Area+=A_i;
		Total_Quality+=q_i;
	}
		// Quality of segmentation
			//Area Coverage Ratio Area/Area post segmented (C)
			//Validity Ratio: regions with A>A_min and edge_size>edge_size_min (R)
			//Simplicity exp(-(N-Ñ)/\Phi). N= current number of nodes; Ñ: expected number of regions	
	
	float Parent_Area = cv::contourArea(Parent_contour);
	float Area_Coverage_Ratio =(Total_Area/Parent_Area);//Regularly 1
	float Validity_Ratio = 1;//All areas are valid
	float Simplicity=1; // regions coincide with user defined
	
	cout << "Area_Coverage_Ratio " << Area_Coverage_Ratio << endl;
	
	float Overall_Quality;
	float lambda=1; //equally weighted

	Overall_Quality = (Area_Coverage_Ratio * Validity_Ratio)/(contours_centroid.size())*Total_Quality + lambda*Simplicity;

	cout <<"C "<<Area_Coverage_Ratio<<" R "<<Validity_Ratio<<" N "<<contours_centroid.size()<<" S "<<Simplicity<<" Q_avg "<<Total_Quality/contours_centroid.size()<<endl;
	cout << "Overall_Quality " << Overall_Quality << endl;

	// Overall Quality
		// CR/N \sum( q_i) +  \lambda exp( -(N-Ñ)/\Phi )   with \lambda 
	
}


//

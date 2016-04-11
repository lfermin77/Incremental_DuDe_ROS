//------------------------------------------------------------------------------
//  Copyright 2010-2015 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#include "gettime.h"
#include <fstream>
#include "main.h"
#include "bpc.h"
#include "dude_cut.h"
#include "diagonal2.h"
#include "polygon.h"
#include "dude_use.h"
#include "SE2d_skeleton.h"
#include "simple_svg_1.0.0.h"
#include <sstream>


using namespace std;


//
//
//
//
//   The MAIN function
//
//
//
//
void prepare_skeleton();
void decompose(vector<c_polygon*>& finalPolygonPieces);
void export_files(vector<c_polygon*>& finalPolygonPieces);
void export_individual_files(vector<c_polygon*>& finalPolygonPieces);
void export_individual_svg_files(vector<c_polygon*>& finalPolygonPieces);
void export_all_svg_files(vector<c_polygon*>& finalPolygonPieces);



int main(int argc, char ** argv) 
{
	if (!parseArg(argc, argv)) 
	{
		printUsage(argv[0]);
		return 1;
	}

	prepare_skeleton();

	//---------------------------------------------------------------
	double start_dude = getTime();

	vector<c_polygon*> finalPolygonPieces; //the result polygons, they can be exported as individual files
	decompose(finalPolygonPieces);

	double end_dude = getTime();
	//---------------------------------------------------------------

	cerr << "- Compute Dual-space Decomposition of "<<dude_param.Pfile<<" with tau "<<dude_param.concavity_tau<<" takes " << end_dude - start_dude
			<< " ms" << endl;

	estimate_COM_R_Box(); //compute the bounding box of all geometries

	//export files
	export_files(finalPolygonPieces);

	/////////////////////////////////////////////////////////////////
	//setup glut/gli
#if GL_RENDERING
	if (draw_options.showGL) {

		glutInit(&argc, argv);
		createGLUI();

		/////////////////////////////////////////////////////////////
		//set camera position
		gli::setCameraPosZ(3 * draw_decoration.R);
		gli::set2DMode(true);
		/////////////////////////////////////////////////////////////
		cerr << "- Press ? to show GUI usage" << endl;
		gli::gliMainLoop();
	}
#endif

	return 0;
}

///////////////////////////////////////////////////////////////////////////
//debug
void save_cut()
{

	ofstream ofile("eid.txt");
	for(vector<c_diagonal>::iterator dit = dude.m_cuts.begin(); dit != dude.m_cuts.end(); ++dit)
	{
		c_diagonal& diag = *dit;
		ofile<<diag.getV1()->getVID()<<" "<<diag.getV2()->getVID()<<endl;
	}
	ofile.close();
}
void loda_cuts(vector<c_polygon*>& finalPolygonPieces)
{
	c_polygon* p1 = new c_polygon();//, p2;
	p1->copy(getP());

	p1->build_all();

	ifstream infile("eid.txt");

	while(!infile.eof())
	{
		int id1, id2;
		infile>>id1; infile>>id2;

		c_diagonal diag;
		diag.setEndPoints((*p1)[id1], (*p1)[id2]);
		dude.m_cuts.push_back(diag);
	}
	infile.close();

	updateMINMAXConcavity(dude);

	//debug
	p1->build_all();
	save_cut();

	if(dude_param.decomposeIteratively){
		cerr<<"iteratively decomposing..."<<endl;
		iterativeDecompose(*p1, dude.m_cuts, dude_param.concavity_tau, finalPolygonPieces, draw_decoration.allAccumulatedCuts, draw_decoration.se, true);
	}
	else
	{
		decomposeMoreTimes(*p1, dude.m_cuts, dude_param.concavity_tau, finalPolygonPieces, draw_decoration.allAccumulatedCuts, draw_decoration.se, true, 0);
	}

	cerr<<"component number "<<finalPolygonPieces.size()<<endl;
	draw_decoration.resultPolygons = finalPolygonPieces;

}
///////////////////////////////////////////////////////////////////////////


void decompose(vector<c_polygon*>& finalPolygonPieces)
{
	//loda_cuts(finalPolygonPieces);
	//return;


	c_polygon* p1 = new c_polygon();//, p2;
	p1->copy(getP());
	//p2.copy(getP());

	{
		dude.build(*p1, dude_param.concavity_tau,true);
		draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), dude.m_PMs.begin(), dude.m_PMs.end());
		draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), dude.hole_PMs.begin(), dude.hole_PMs.end());
		updateMINMAXConcavity(dude);

		//debug
		p1->build_all();
		save_cut();

		if(dude_param.decomposeIteratively){
			cerr<<"iteratively decomposing..."<<endl;
			iterativeDecompose(*p1, dude.m_cuts, dude_param.concavity_tau, finalPolygonPieces, draw_decoration.allAccumulatedCuts, draw_decoration.se, true);
		}
		else
		{
			decomposeMoreTimes(*p1, dude.m_cuts, dude_param.concavity_tau, finalPolygonPieces, draw_decoration.allAccumulatedCuts, draw_decoration.se, true, 0);
		}

		cerr<<"component number "<<finalPolygonPieces.size()<<endl;
		draw_decoration.resultPolygons = finalPolygonPieces;
	}

}

void export_files(vector<c_polygon*>& finalPolygonPieces)
{
	//export decomposed polygons
	if (dude_param.export_decomp_polys)
		export_individual_files(finalPolygonPieces);

	if (dude_param.export_decomp_polys_svg)
	{
		export_all_svg_files(finalPolygonPieces);
		export_individual_svg_files(finalPolygonPieces);
	}

	////////////////////////////////////////////////////////////////
	//export the generated cut
	if(draw_options.outputSegFile)
		saveDiagonals(draw_decoration.allAccumulatedCuts, dude_param.Pfile + ".seg.dude2d");


	////////////////////////////////////////////////////////////////
	//export options
#if PS_RENDERING
	if (draw_options.savsPS) {
		string filename = dude_param.Pfile;
		if(draw_options.showPolygonPieceForPS)
		{
			filename += "_pp";
		}
		if(draw_options.showSkeletonForPS){
			filename += "_pske";
		}
		//if(draw_options.visulizeSegment){
		//	filename += "_" + draw_options.vizSource;
		//}

		filename += ".ps";
		save2PS(filename);
		cout << "- Save " << filename << endl;
	}
#endif
}


void prepare_skeleton()
{
	//prepare for extract skeletons
    ExtracSkeleton * m_ES = ES_Factory::create_ES("PA");
    draw_decoration.se.setExtracSkeleton(m_ES);
    draw_decoration.se.setQualityMeasure(NULL);
    draw_decoration.se.begin();
}

void export_individual_files(vector<c_polygon*>& finalPolygonPieces)
{
	string basename = dude_param.Pfile;
	int psize = finalPolygonPieces.size();
	for(int i = 0; i < psize; i++)
	{
		stringstream ss;
		ss << basename.c_str() << "_" << setfill('0') << setw(5) << i << ".poly";
		string sname = ss.str();

		ofstream ofile(sname.c_str());
		ofile<<(*finalPolygonPieces[i]);
		ofile.close();
		cout << "- Saved: " << sname << endl;
	}
}

void export_individual_svg_files(vector<c_polygon*>& finalPolygonPieces)
{
	string basename = dude_param.Pfile;
	int psize = finalPolygonPieces.size();

	c_polygon & P = getP();
	double * bbox = P.getBBox();
	double width  = bbox[1] - bbox[0];
	double height = bbox[3] - bbox[2];
	double stroke_width = width / 200;
	svg::Dimensions dimensions(800, 800);
	svg::Dimensions svg_viewbox_dim(width + stroke_width*2, height + stroke_width*2);
	svg::Point svg_viewbox_org(bbox[0] - stroke_width, height - bbox[3] + stroke_width);

	svg::Polygon original(svg::Fill(svg::Color::Yellow), svg::Stroke(stroke_width / 2, svg::Color::Black));
	P.toSVG(original);

	for (int i = 0; i < psize; i++)
	{
		stringstream ss;
		ss << basename.c_str() << "_" << setfill('0') << setw(5) << i << ".svg";
		string sname = ss.str();
		svg::Document doc(sname, svg::Layout(dimensions, svg::Layout::BottomLeft, svg_viewbox_dim, svg_viewbox_org));
		svg::Polygon p_i(svg::Fill(svg::Color::Lime), svg::Stroke(stroke_width, svg::Color::Black));
		finalPolygonPieces[i]->toSVG(p_i);
		doc << original << p_i;
		doc.save();
		cout << "- Saved: " << sname << endl;
	}
}



void export_all_svg_files(vector<c_polygon*>& finalPolygonPieces)
{
	string basename = dude_param.Pfile;
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


//------------------------------------------------------------------------------
//  Copyright 2010-2015 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _DUDE_MAIN_H_
#define _DUDE_MAIN_H_
#pragma once

#include "compile_option.h"

//
//
//
// This is the main header file
// Functions defined in this file are mainly for text parsing and rendering 
//
//
//
#include <float.h>
#include <time.h>
#include <fstream>
#include <map>
using namespace std;


#include "Quaternion.h"
#include "polygon.h"
#include "diagonal2.h"
#include "polyline.h"
#include "dude_use.h"
#include "gettime.h"
#include "holediag.h"
#include "dude_param.h"

//----------------------------------------------------------------------------
//draw options
#include "draw_option.h"
#include "draw_decoration.h"
Draw_Options draw_options;
Draw_Decoration draw_decoration;


#if GL_RENDERING
#include <GL/gli.h>
#include <GL/gliDump.h>
#include <GL/gl2ps.h>
#include "draw.h"
#include "drawPS.h"

double average=0; //used to control the radius of dots

#endif

//-----------------------------------------------------------------------------
// INPUTS

Dude_Param dude_param;


//-----------------------------------------------------------------------------
// main dude objects
c_dude dude;


void computeCOM_R();


//-----------------------------------------------------------------------------
// glui objects
#if GL_RENDERING
GLUI_EditText* editTau = NULL;
GLUI_Listbox * vipv_box = NULL;
#endif

//-----------------------------------------------------------------------------
//read poly function
bool readPoly(const string& name, c_polygon& poly) {
	ifstream fin(name.c_str());
	if (!fin.good()) {
		cerr << "! Error: Cannot open file (" << name << ")" << endl;
		return false;
	}

	fin >> poly;
	fin.close();

	return true;
}

#if GL_RENDERING
inline void createVIP_vertex_list() {
	const list<c_BPC *>& bpcs = dude.getBPS();
	typedef list<c_BPC *>::const_iterator IT;
	for (IT i = bpcs.begin(); i != bpcs.end(); i++) {
		const list<ply_vertex *>& pms = (*i)->getConcavities();
		for (list<ply_vertex *>::const_iterator j = pms.begin(); j != pms.end();
				j++) {
			draw_decoration.VIP_vertices.push_back(*j);
		}
	}

	const list<ply_vertex *>& holefp = dude.hole_PMs;
	//vector<ply_vertex*>::iterator pite = holeMedialAxisPoints.begin();
	//for(;pite!=holeMedialAxisPoints.end();++pite)
	for (list<ply_vertex *>::const_iterator i = holefp.begin();
			i != holefp.end(); i++) {
		if ((*i)->getExtra().isPM() == false
				|| (*i)->getExtra().diagonals.empty())
			continue;
		draw_decoration.VIP_vertices.push_back(*i);
	}
}

void selectVIPV(int tmp) {
	if (draw_decoration.selectedVIPVertex < 0
			|| draw_decoration.selectedVIPVertex > (int) (draw_decoration.VIP_vertices.size()))
		draw_decoration.g_selected_PM = NULL;
	else
		draw_decoration.g_selected_PM = draw_decoration.VIP_vertices[draw_decoration.selectedVIPVertex];
}

//-----------------------------------------------------------------------------
//functions used in rendering
void Display();
bool InitGL();
void Reshape(int w, int h);
void Keyboard(unsigned char key, int x, int y);
void Mouse(int button, int state, int x, int y);

#endif 

//-----------------------------------------------------------------------------
bool parseArg(int argc, char ** argv) 
{
	if (argc < 2) return false;

	for (int i = 1; i < argc; i++) 
	{
		if (argv[i][0] == '-') {
			//if(strcmp(argv[i],"-o")==0) output_filename=argv[++i];
			//else
			if (strcmp(argv[i], "-g") == 0)
			{
				draw_options.showGL = false;
			}
			else if (strcmp(argv[i], "-P") == 0)
			{
				draw_options.neg_P = true;
				//else if(strcmp(argv[i],"-k")==0) num = atoi(argv[++i]);
			}
			else if (strcmp(argv[i], "-ps") == 0) 
			{
				draw_options.savsPS = true;
				draw_options.showConcavity = true;
			} 
			else if (strcmp(argv[i], "-viz") == 0)
			{
				draw_options.visulizeSegment = true;
				draw_options.vizSource = string(argv[++i]);
				draw_options.useDude2d = (draw_options.vizSource == "internal");

				draw_options.savsPS = true;
				draw_options.showConcavity = true;
			}
			else if (strcmp(argv[i], "-ske") == 0)
			{
				draw_options.showSkeleton = true;
			}
			//else if(strcmp(argv[i],"-j")==0) convertToJSON=true;
			else if (strcmp(argv[i], "-tau") == 0)
			{
				dude_param.concavity_tau = atof(argv[++i]);
			}
			else if (strcmp(argv[i], "-os") == 0) // output seg file
			{
				draw_options.outputSegFile = true;
			} 
			else if (strcmp(argv[i], "-ite") == 0) 
			{
				//iterateTimes = atoi(argv[++i]);
				dude_param.decomposeIteratively = false; //true;
			} 
			else if (strcmp(argv[i], "-pp") == 0) 
			{
				draw_options.showPolygonPieceForPS = true;

				draw_options.savsPS = true;
				draw_options.showConcavity = true;
			}
			else if (strcmp(argv[i], "-pske") == 0)
			{
				draw_options.showSkeletonForPS = true;

				draw_options.savsPS = true;
				draw_options.showConcavity = true;
			}
			else if(strcmp(argv[i], "-polys") == 0)
			{
				dude_param.export_decomp_polys = true;
			}
			else if (strcmp(argv[i], "-svg") == 0)
			{
				dude_param.export_decomp_polys_svg = true;
			}
			else
				return false; //unknown
		} 
		else
		{
			dude_param.Pfile = argv[i];
		}
	}

	if (dude_param.Pfile.empty())
		return false;

	if (!readPoly(dude_param.Pfile, getP()))
		return false;

	//normalize P
	getP().normalize();

	if (dude_param.rotate_angle != 0)
		getP().rotate(dude_param.rotate_angle * PI / 180);
	if (draw_options.neg_P)
		getP().negate();

	c_ply& outply = getP().front();
	if (outply.getType() != c_ply::POUT) {
		for (c_polygon::iterator pit = getP().begin(); pit != getP().end();
				++pit) {
			c_ply& tply = *pit;
			if (tply.getType() == c_ply::POUT) {
				outply = *pit;
				break;
			}
		}
	}

	double outRadius = outply.getRadius();
	for (c_polygon::iterator pit = getP().begin(); pit != getP().end(); ++pit) {
		c_ply& tply = *pit;
		if (tply.getType() == c_ply::POUT)
			continue;
		if (abs(tply.getArea()) < 0.01 * outRadius * outRadius) {
			tply.canBeIgnored = true;
			//cout<<"ignored"<<endl;
		}
	}

	return true;
}

void printUsage(char * name) 
{
	int offset = 15;
	cerr << "Usage: " << name << " [options] P.poly" // Q.poly \n"
			<< "\noptions:\n\n";

#if GL_RENDERING
	cerr << left << setw(offset) << "-g" << " disable openGL visualization\n";
#endif

	cerr << left << setw(offset) << "-o file"
			<< " dump decompoisition to file\n";

	//cerr << left << setw(offset) << "-j" << " convert to json file\n";

	cerr << left << setw(offset) << "-k [value]"
			<< " create k-number of decomposition\n";
	cerr << left << setw(offset) << "-tau [value]" << " tolerable concavity\n";
	
	//cerr << left << setw(offset) << "-com [file.txt]"
	//		<< " to compare the segmenation with a segmentation from file\n";
	
	cerr << left << setw(offset) << "-ite"
			<< " decompse once\n";

	cerr << left << setw(offset) << "-ske"
		<< " build skeleton after decomposition\n";

#if PS_RENDERING
	cerr << left << setw(offset) << "-ps" << " saves to ps file\n";
	cerr << left << setw(offset) << "-pske"
		<< " save skeleton ps file\n";
	cerr << left << setw(offset) << "-pp" << " saves polygon pieces to ps file\n";
#endif

	cerr<<left <<setw(offset)<<"-polys"
			<<" export decomposed polygons into poly files\n";


	cerr << left << setw(offset) << "-svg"
		<< " export decomposed polygons into svg files\n";


	cerr << flush;
}

#if GL_RENDERING

void printGUIUsage() 
{
	static int print_count = 1;
	int offset = 15;
	cerr << "GUI Usage:\n";

	cout << left << setw(offset) << "p:"
			<< "show/hide P, the rotating polygon\n";
	cout << left << setw(offset) << "n:" << "show/hide normal direction \n";
	cout << left << setw(offset) << "r:" << "reset everything\n";
	cout << left << setw(offset) << "9:" << "on/off dumping pdf images\n";
	cout << left << setw(offset) << "0:" << "on/off dumping ppm images\n";
	cout << left << setw(offset) << "c:" << "show/hide concavities\n";
	cout << left << setw(offset) << "d:" << "show/hide decomposition\n";
	cout << left << setw(offset) << "b:" << "show/hide bridges\n";
	cout << left << setw(offset) << "w:"
			<< "show/hide potential cuts/cutsets\n";
	cout << left << setw(offset) << "G:" << "show/hide simplified polygon\n";
	cout << left << setw(offset) << "P:" << "print to a PS file\n";
	cout << left << setw(offset) << "5:" << "zoom in\n";
	cout << left << setw(offset) << "6:" << "zoom out\n";
	cout << left << setw(offset) << "C:"
			<< "show all potential cuts one by one,increase the index by pressing I\n";
	cout << left << setw(offset) << "S:"
			<< "show all simplified polygons one by one,increase the index by pressing I\n";
	cout << left << setw(offset) << "B:"
			<< "show all bridges one by one,increase the index by pressing I\n";
	cout << left << setw(offset) << "I:"
			<< "increase the index of showing elements one by one\n";
	cout << left << setw(offset) << "Y:"
			<< "show all the plys generated during the process one by one,\n"
			<< left << setw(offset) << " "
			<< "increase the index by pressing I\n";
	cout << left << setw(offset) << "R:"
			<< "show all the generated rest plys excluding child pockets one by one,\n"
			<< left << setw(offset) << " "
			<< "increase the index by pressing I\n";
	cout << left << setw(offset) << "H:"
			<< "show all the generated parent pocket and its children pockets one by one, \n"
			<< left << setw(offset) << " "
			<< "increase the index by pressing I\n";
	cout << left << setw(offset) << "Q:" << "show cutted polygon pieces\n";
	cout << left << setw(offset) << "U:" << "show all user cuts\n";
	cout << left << setw(offset) << "L:"
			<< "show all user cuts in clustering.\n";
	cout << left << setw(offset) << "u:" << "show rep user cuts\n";
	cout << left << setw(offset) << "E:" << "show skeleton\n";
	cout << left << setw(offset) << "t:" << "show cuts to compare with\n";
	cout << left << setw(offset) << "arrows:" << "translate\n";
	cout << left << setw(offset) << "ctrl-click:"
			<< "select a concavity vertex\n";
	cout << left << setw(offset) << "?:"
			<< "show this message or show info of selected vertex\n";
	cout << left << setw(offset) << "------------"
			<< "This message has been printed: " << print_count++ << " time(s)"
			<< endl;
}

#endif 

//-----------------------------------------------------------------------------
void estimate_COM_R_Box() 
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

void DuDe() 
{

#if GL_RENDERING
	if (editTau != NULL) 
	{
#if 0
        accumulatedCuts.clear();
        allSimplifiedPolys.clear();
        allBridges.clear();
        allPocPlys.clear();
        allExPocPlys.clear();
        allExPocBridges.clear();
        allExPocApproxs.clear();
        parentPck.clear();
        childrenPck.clear();
#endif

		dude_param.concavity_tau = editTau->get_float_val();
	}
#endif

	cout << "- Dual-space Decomposition with tau=" << dude_param.concavity_tau << endl;
	double start_dude = getTime();
	dude = c_dude();
	dude.build(getP(), dude_param.concavity_tau, true);
	draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), dude.m_PMs.begin(),
			dude.m_PMs.end());

	//c_polygon cplygon;
	//cplygon.copy(getP());

	//dude_use=c_dude_use();
	//dude_use.addInitialPolygon(cplygon);
	//dude_use.startDecompose(concavity_tau);

	//resultPolygons = dude_use.todo_list;

	//double end_dude=getTime();
	//cout<<"- Compute Dual-space Decomposition takes "<<end_dude-start_dude<<" ms"<<endl;
	//glutPostRedisplay();
}

#if GL_RENDERING
//-----------------------------------------------------------------------------
//
//
//
//  Open GL stuff below
//
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void createGLUI() {
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(480, 480);
	glutInitWindowPosition(50, 50);

	string title = "dual decomposition (dude) 2d: " + dude_param.Pfile;
	int windowID = glutCreateWindow((char*) title.c_str());

	//glutCreateWindow( "dude 2d" );

	InitGL();
	gli::gliInit();
	gli::gliDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	gli::gliMouseFunc(Mouse);

//    GLUI_Master.set_glutIdleFunc( NULL );
//    GLUI_Master.set_glutKeyboardFunc(Keyboard);
//    GLUI_Master.set_glutMouseFunc(Mouse);

	//GLUI_Master.set_glutSpecialFunc(SpecialKey);
	GLUI *glui = GLUI_Master.create_glui(title.c_str(), 0, 50 + 500, 50);

	// SETTING UP THE CONTROL PANEL:
	GLUI_Panel * top_panel = glui->add_panel("DUDE 2D Controls");

	editTau = glui->add_edittext_to_panel(top_panel, "concavity tau:",
			GLUI_EDITTEXT_FLOAT);
	editTau->set_float_limits(1e-10, 1e10, GLUI_LIMIT_CLAMP);
	editTau->set_float_val(dude_param.concavity_tau);
	//ostringstream tmpss;
	//tmpss << concavity_tau;
	//editTau->set_text((char*)tmpss.str().c_str());
	GLUI_Button* buttonRun = glui->add_button_to_panel(top_panel, "Run", -1,
			(GLUI_Update_CB) DuDe);
	buttonRun->set_name("Decompose Again");

	glui->add_checkbox_to_panel(top_panel, "Show Polygon (p)", &draw_options.showP, -1,
			(GLUI_Update_CB) Display)->set_int_val(draw_options.showP);
	glui->add_checkbox_to_panel(top_panel, "Show Concavity (c)", &draw_options.showConcavity,
			-1, (GLUI_Update_CB) Display)->set_int_val(draw_options.showConcavity);
	glui->add_checkbox_to_panel(top_panel, "Show Bridges (b)", &draw_options.showBridges, -1,
			(GLUI_Update_CB) Display)->set_int_val(draw_options.showBridges);
	glui->add_checkbox_to_panel(top_panel, "Show Decomposition (d)",
			&draw_options.showDecompose, -1, (GLUI_Update_CB) Display)->set_int_val(
					draw_options.showDecompose);
	glui->add_checkbox_to_panel(top_panel, "Show Constraints (G)",
			&draw_options.showConstraints, -1, (GLUI_Update_CB) Display)->set_int_val(
					draw_options.showConstraints);
	glui->add_checkbox_to_panel(top_panel, "Show Potential Cuts (w)", &draw_options.showCuts,
			-1, (GLUI_Update_CB) Display)->set_int_val(draw_options.showCuts);
	//glui->add_checkbox_to_panel(top_panel,"Show All Cuts (C)", &showAllCuts, -1, (GLUI_Update_CB)Display)->set_int_val(showAllCuts);
	//glui->add_checkbox_to_panel(top_panel,"Show All SimPolygons (S)", &showAllSimPolygons, -1, (GLUI_Update_CB)Display)->set_int_val(showAllSimPolygons);
	glui->add_checkbox_to_panel(top_panel, "Show Normal directions (n)",
			&draw_options.showNormal, -1, (GLUI_Update_CB) Display)->set_int_val(draw_options.showNormal);
	glui->add_checkbox_to_panel(top_panel, "Show Polygon Pieces (Q)",
			&draw_options.showPolygonPieces, -1, (GLUI_Update_CB) Display)->set_int_val(
			draw_options.showPolygonPieces);
	glui->add_checkbox_to_panel(top_panel, "Show Circle with Diameter tau",
			&draw_options.showTauCircle, -1, (GLUI_Update_CB) Display)->set_int_val(
			draw_options.showTauCircle);
	glui->add_checkbox_to_panel(top_panel, "Show Vertex ID", &draw_options.showVertexID, -1,
			(GLUI_Update_CB) Display)->set_int_val(draw_options.showVertexID);

	glui->add_checkbox_to_panel(top_panel, "Dump PDF files (9)", &draw_options.savePDF, -1,
			(GLUI_Update_CB) Display)->set_int_val(draw_options.savePDF);
	glui->add_checkbox_to_panel(top_panel, "Dump PPM files (0)", &draw_options.saveImg, -1,
			(GLUI_Update_CB) Display)->set_int_val(draw_options.saveImg);

	///////////////////////////////////////////////////////////////
	glui->add_checkbox_to_panel(top_panel, "Show few (d)",
			&draw_options.showFewCut, -1, (GLUI_Update_CB) Display)->set_int_val(
					draw_options.showFewCut);

	//////////////////////////////////////////////////////////////



	//
	createVIP_vertex_list();
	vipv_box = glui->add_listbox_to_panel(top_panel, "select vertex ",
			&draw_decoration.selectedVIPVertex, -1, (GLUI_Update_CB) selectVIPV);
	vipv_box->add_item(-1, "none");
	int vipvsize = draw_decoration.VIP_vertices.size();
	for (int i = 0; i < vipvsize; i++) {
		char tmp[24];
		sprintf(tmp, "vertex %d", int(draw_decoration.VIP_vertices[i]->getVID()));
		vipv_box->add_item(i, tmp);
	}

	glui->set_main_gfx_window(windowID);
}

//-----------------------------------------------------------------------------
void save_PDF(const string& pdf_name) {
	//////////////////////////////////////////////////////////////////////////////
	FILE *fp = fopen(pdf_name.c_str(), "wb");
	GLint buffsize = 1024 * 1024; //, state = GL2PS_OVERFLOW;
	GLint viewport[4];

	glGetIntegerv(GL_VIEWPORT, viewport);
	gl2psBeginPage("mksum 2d", "MASC Group@GMU", viewport,
	GL2PS_PDF/*GL2PS_EPS*/, GL2PS_BSP_SORT,
			GL2PS_SILENT | GL2PS_POLYGON_OFFSET_FILL |
			GL2PS_NO_BLENDING | GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT
					| GL2PS_COMPRESS |
					GL2PS_TIGHT_BOUNDING_BOX | GL2PS_USE_CURRENT_VIEWPORT,
			GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, NULL);

	gl2psPointSize(1);
	gl2psLineWidth(1);
	drawAll();

	gl2psEndPage();

	fclose(fp);
}

//-----------------------------------------------------------------------------
void Display(void) {

	//Init Draw
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	static GLfloat light_position1[] = { 500, 500.0f, -500.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
	static GLfloat light_position2[] = { 500, 500, 500.0f, 1.0f };
	glLightfv(GL_LIGHT1, GL_POSITION, light_position2);

	glMatrixMode( GL_PROJECTION);
	glLoadIdentity();

	//REAL scale = gli::getScale();
	float scale = gli::getScale();
	glOrtho((draw_decoration.box[0] - draw_decoration.COM[0]) * scale + draw_decoration.COM[0],
			(draw_decoration.box[1] - draw_decoration.COM[0]) * scale + draw_decoration.COM[0],
			(draw_decoration.box[2] - draw_decoration.COM[1]) * scale + draw_decoration.COM[1],
			(draw_decoration.box[3] - draw_decoration.COM[1]) * scale + draw_decoration.COM[1], -1000, 1000);

	glMatrixMode( GL_MODELVIEW);
	glLoadIdentity();

	gli::gliTranslate();

	//display MKsum and Convolution
	drawAll();

	//dump to images
	if (draw_options.saveImg || draw_options.savePDF) {
		//dump
		char number[64];
		sprintf(number, "_%04d", draw_decoration.current_imgID++);
		string filename = dude_param.Pfile + number;

		if (draw_options.saveImg) {
			filename = filename + ".ppm";
			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT, viewport);
			dump(filename.c_str(), viewport[2], viewport[3]);

		}

		if (draw_options.savePDF) {
			filename = filename + ".ps";
			//save_PDF(filename);
			save2PS(filename);
		}

		cerr << "- Save Image : " << filename << endl;
	}
}

//-----------------------------------------------------------------------------
// regular openGL callback functions
bool InitGL() {
	// *Antialias*
	glEnable( GL_LINE_SMOOTH);
	glEnable( GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint( GL_LINE_SMOOTH_HINT, GL_NICEST);

	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_BACK);

	// others
	glEnable( GL_DEPTH_TEST);
	glClearColor(1, 1, 1, 0);
	//glClearColor( 0,0,0,0 );

	//Let's have light!
	GLfloat Diffuse[] = { 0.9f, 0.9f, 0.9f, 1.0f };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	GLfloat WhiteLight[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, WhiteLight);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, WhiteLight);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	return true;
}

void Reshape(int w, int h) {
	//glViewport( 0, 0, (GLsizei)w, (GLsizei)h );
	if (w > h)
		glViewport(0, 0, (GLsizei) w, (GLsizei) w);
	else
		glViewport(0, 0, (GLsizei) h, (GLsizei) h);

	glMatrixMode( GL_PROJECTION);
	glLoadIdentity();
	glOrtho(draw_decoration.box[0], draw_decoration.box[1], draw_decoration.box[2], draw_decoration.box[3], -1000, 1000);
	glMatrixMode( GL_MODELVIEW);
	glLoadIdentity();
}

void resetCamera() {
	//reset camera
	gli::setScale(1);
	gli::setCameraPosZ(draw_decoration.R * 3);
	gli::setCameraPosX(0);
	gli::setCameraPosY(0);
	gli::setAzim(0);
	gli::setElev(0);
}

void Keyboard(unsigned char key, int x, int y) {
	string PSfilename;

	// find closest colorPt3D if ctrl is pressed...
	switch (key) {
	case 27:
		exit(0);
	case 'p':
		draw_options.showP = !draw_options.showP;
		break;
	case '9':
		draw_options.savePDF = !draw_options.savePDF;
		break;  //turn on/off savePDF
	case '0':
		draw_options.saveImg = !draw_options.saveImg;
		break;  //turn on/off image dumping
	case 'n':
		draw_options.showNormal = !draw_options.showNormal;
		break; //turn on/off normal display
	case 'c':
		draw_options.showConcavity = !draw_options.showConcavity;
		break;
	case 'b':
		draw_options.showBridges = !draw_options.showBridges;
		break;
	case 'w':
		draw_options.showCuts = !draw_options.showCuts;
		break;
	case 'd':
		draw_options.showDecompose = !draw_options.showDecompose;
		break;
	case 'G':
		draw_options.showConstraints = !draw_options.showConstraints;
		break;
	case 'L':
		draw_options.showAllUserCutsInClustering = !draw_options.showAllUserCutsInClustering;
		break;
	case 'U':
		draw_options.showAllUserCuts = !draw_options.showAllUserCuts;
		break;
	case 'u':
		draw_options.showRepUserCuts = !draw_options.showRepUserCuts;
		break;
	case 'E':
		draw_options.showSkeleton = !draw_options.showSkeleton;
		break;
	case 't':
		draw_options.showCompCuts = !draw_options.showCompCuts;
		break;
		//case 'C' : showAllCuts = !showAllCuts; break;
		//case 'S' : showAllSimPolygons = !showAllSimPolygons; break;
	case 'B':
		draw_options.showAllBridges = !draw_options.showAllBridges;
		break;
		//case 'I' : showElementIdx++;break;
		//case 'Y' : showAllPlys = !showAllPlys; break;
		//case 'R' : showExPocPlys = !showExPocPlys; break;
		//case 'H' : showParentChildPck = !showParentChildPck; break;
	case 'Q':
		draw_options.showPolygonPieces = !draw_options.showPolygonPieces;
		break;

	case 'P': //print to a PS file
		PSfilename = dude_param.Pfile + ".ps";
		save2PS(PSfilename);
		cout << "- Save " << PSfilename << endl;
		break;
	case 'r':
		resetCamera();
		break;
	case '-':
		draw_decoration.normal_length /= 2;
		break;
	case '+':
		draw_decoration.normal_length *= 2;
		break;
	case '5':
		gli::setScale(gli::getScale() * 0.95);
		break;
	case '6':
		gli::setScale(gli::getScale() * 1.05);
		break;
	case '?':
		if (draw_decoration.g_selected_PM == NULL)
			printGUIUsage();
		else {
			vector<c_cutset>& csets = draw_decoration.g_selected_PM->getExtra().cutsets;
			uint cset_size = csets.size();
			cout << "PM[" << draw_decoration.g_selected_PM->getVID() << "] has " << cset_size
					<< " cutsets, concavity="
					<< draw_decoration.g_selected_PM->getExtra().concavity << endl;
			for (uint j = 0; j < cset_size; j++) {
				cout << csets[j] << endl;
			}
			cout << "\t dihedral Pre Vid="
					<< draw_decoration.g_selected_PM->getExtra().getDihedralPre()->getVID()
					<< " dihedral Next Vid="
					<< draw_decoration.g_selected_PM->getExtra().getDihedralNext()->getVID()
					<< endl;
		}

		break;
	}

	GLUI_Master.sync_live_all();
	glutPostRedisplay();
}

//
//
// selecting a pocket minimum...
//
//
void Mouse(int button, int state, int x, int y) {
	c_dude& ap = dude;
	const list<c_BPC *>& bpcs = ap.getBPS();
	float pR = getP().front().getRadius();

	typedef list<c_BPC *>::const_iterator IT;

	static float smallest_concavity = FLT_MAX;
	static float largest_concavity = -FLT_MAX;

	if (smallest_concavity == FLT_MAX) {
		for (IT i = bpcs.begin(); i != bpcs.end(); i++) {
			const list<ply_vertex *>& pms = (*i)->getConcavities();
			for (list<ply_vertex *>::const_iterator j = pms.begin();
					j != pms.end(); j++) {
				float c = (*j)->getExtra().concavity;
				if (c < smallest_concavity) {
					smallest_concavity = c;
				}
				if (c > largest_concavity) {
					largest_concavity = c;
				}
			}
		}

		largest_concavity /= pR;
		smallest_concavity /= pR;
	}

	//control needs to be pressed to selelect nodes
	if (state == GLUT_UP && glutGetModifiers() == GLUT_ACTIVE_CTRL) {
		draw_decoration.g_selected_PM = NULL;

		int viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		const float * cam_pos = gli::getCameraPos();
		float scale = gli::getScale();

		Point2d m(x, viewport[3] - y);

		double scaled_box[4] = { draw_decoration.box[0], draw_decoration.box[1], draw_decoration.box[2], draw_decoration.box[3] };
		scaled_box[0] = (draw_decoration.box[0] - draw_decoration.COM[0]) * scale + draw_decoration.COM[0];
		scaled_box[1] = (draw_decoration.box[1] - draw_decoration.COM[0]) * scale + draw_decoration.COM[0];
		scaled_box[2] = (draw_decoration.box[2] - draw_decoration.COM[1]) * scale + draw_decoration.COM[1];
		scaled_box[3] = (draw_decoration.box[3] - draw_decoration.COM[1]) * scale + draw_decoration.COM[1];

		for (vector<ply_vertex *>::const_iterator i = draw_decoration.VIP_vertices.begin();
				i != draw_decoration.VIP_vertices.end(); i++) {
			float c = (*i)->getExtra().concavity / pR;
			float r = average * (c - smallest_concavity + 0.2)
					/ (largest_concavity - smallest_concavity + 0.2) * 2;
			Point2d pt = (*i)->getPos();
			pt[0] = (pt[0] - scaled_box[0] + cam_pos[0])
					* (viewport[2] / (scaled_box[1] - scaled_box[0]));
			pt[1] = (pt[1] - scaled_box[2] - cam_pos[1])
					* (viewport[3] / (scaled_box[3] - scaled_box[2]));
			r = (r) * (viewport[2] / (scaled_box[1] - scaled_box[0]));

			double dist = (pt - m).norm();
			if (dist < r) {
				draw_decoration.g_selected_PM = *i;
				if (vipv_box != NULL) {
					int id = (i - draw_decoration.VIP_vertices.begin());
					vipv_box->do_selection(id);
					GLUI_Master.sync_live_all();
				}
				glutPostRedisplay();
				return;
			}
		} //for i

	} //if pressed the right key/button
}

#endif // GL_RENDERING

#endif //_DUDE_MAIN_H_


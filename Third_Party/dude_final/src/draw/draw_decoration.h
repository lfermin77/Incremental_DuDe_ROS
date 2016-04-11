/*
 * draw_decoration.h
 *
 *  Created on: Jan 26, 2015
 *      Author: guilin
 */

#ifndef DRAW_DECORATION_H_
#define DRAW_DECORATION_H_

#include <vector>
#include <float.h>

#include "diagonal2.h"
#include "polygon.h"
#include "SE2d.h"

using namespace std;

struct Draw_Decoration
{
	Draw_Decoration() : resultPolygons(tvec)
	{
		MAXCONCAVITY = -FLT_MAX;
		MINCONCAVITY = FLT_MAX;

		current_imgID = 0;
		R = 0;

		box[0] = FLT_MAX; box[1] = - FLT_MAX;
		box[2] = FLT_MAX; box[3] = - FLT_MAX;

		normal_length = 1; //this controls the scale for the edge normal
		selectedVIPVertex = -1;
		g_selected_PM = NULL;
	}

	//--------data generated along the way-------------------

	vector<c_polygon*> tvec;
	vector<c_polygon*>& resultPolygons;

	//this holds a list of important vertices
	vector<ply_vertex *> VIP_vertices;

	//record all the cuts
	vector<c_diagonal> allAccumulatedCuts;

	//record all the holePMs
	vector<ply_vertex*> allFeaturePMs;
	vector<float> allFeatureConcavities;

	// for comparison
	vector<c_diagonal> repDiags;				//Representative cuts from human
	vector<vector<c_diagonal> > userDiags;		//All human segments
	vector<c_diagonal> compDiags;				//diags to comparison

	c_diagonal holecutRoot;

	//for testing
	vector<pair<Point2d, Point2d> > holeCutTreeLines;

	float MAXCONCAVITY;
	float MINCONCAVITY;


	//-----------------------------------------------------

	SE2d se;          //extract skeleton


	//-----------------------------------------------------------------------------
	// Intermediate data
	unsigned int current_imgID; //id for dumping images
	double R;              //radius
	Point<double, 2> COM;     //center of mass
	double box[4]; //min/max x, min/max y
	double normal_length; //this controls the scale for the edge normal

	int selectedVIPVertex;
	ply_vertex * g_selected_PM;
};


#endif /* DRAW_DECORATION_H_ */

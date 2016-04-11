#ifndef _DUDE_CUT_H_
#define _DUDE_CUT_H_

#include "polygon.h"

bool checkPlyContainPnt(c_ply& targetPly,ply_vertex* targetPnt);

/**
 * split edges in the cut.
 */
void addDiagnal( ply_vertex* v1, ply_vertex * v2 );

void markParentPly(c_ply* p,c_polygon* polygon);


void markParentPly(c_polygon* P);

////////////////////////////////////////////
///
///Both of the vertices are on the out boundary
//split the out boundary into two pieces
///
////////////////////////////////////////////
bool cutPolys(pair<c_polygon*,c_polygon*>& polygons,c_diagonal& cut_l,vector<c_diagonal>& allCuts,vector<c_diagonal>::iterator tmpIte,bool& p1hasmorecut,bool& p2hasmorecut);


///////////////////////////////////////////////
//the new cut polys
bool cutPolys_new(pair<c_polygon*,c_polygon*>& polygons,c_diagonal& cut_l,vector<c_diagonal>& allCuts,
			  vector<c_diagonal>::iterator tmpIte,bool& p1hasmorecut,bool& p2hasmorecut);



///////////////////////////////////////////////////////////////////////////////
//
//  For the hole boundary
//	at least one of the vertices is on the hole
///////////////////////////////////////////////////////////////////////////////

void merge2Holes(c_diagonal& cut,vector<c_diagonal>& allCuts,vector<c_diagonal>::iterator tmpIte,c_ply* result_ply, bool& p1hasmorecut,bool& p2hasmorecut);



////////////////////////////////////////////////
// for the cuts from same hole boundary
//
///////////////////////////////////////////////
c_polygon* cutHole(c_diagonal& cut_l,vector<c_diagonal>& allCuts,
		  vector<c_diagonal>::iterator tmpIte);

void test_outbd_num(c_polygon& polygon);

#endif

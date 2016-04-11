#ifndef _HOLECUTCONCAVITY_H_
#define _HOLECUTCONCAVITY_H_

#include "polygon.h"
#include "polyline.h"
#include "holediag.h"
using namespace std;

//#ifndef _CUT_POINTS_
//#define _CUT_POINTS_
//static vector<ply_vertex*> cutPoints;
//#endif

c_plyline list2polyline( const list<ply_vertex*>& vlist );

//collect the end points of all cuts
void collectCutPoints(const vector<c_diagonal>&cuts,vector<ply_vertex*>& collectList);

//check whether tmpV is a vertex of a certain cut
bool checkCutPoint(vector<ply_vertex*>& collectList,ply_vertex* tmpV);

bool checkListContainPnt(list<ply_vertex*>& vlist,ply_vertex* tmpV);


//provide a function to select a root diagonal from the cut for hole that can minimize the depth of hierarchy
HoleDiagonal* getCutDiagRoot(const vector<c_diagonal>& holecuts);

//construct the hierarchical concavity for the cut diagonals
void constructHierarchyConcavity(HoleDiagonal* root);

void hierarchyToList(HoleDiagonal* root, vector<HoleDiagonal*>& holeDiagVec);

#endif

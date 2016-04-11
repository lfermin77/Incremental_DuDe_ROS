#ifndef _DUDE_USE_H_
#define _DUDE_USE_H_

#include "dude.h"
#include "dude_cut.h"
#include "polygon.h"
#include "SE2d.h"
#include <vector>


#define DECOMPOSEMOTETIMENUM 1

using namespace std;


class c_dude_use
{
public:
	vector<c_polygon*> todo_list;

	c_dude_use();

	void destory();

	void addInitialPolygon(c_polygon& cpolygon);

	//use the cuts to cut the 
	void decomposePolygonUsingCuts(c_polygon& cpolygon, vector<c_diagonal>& allCuts, SE2d& se, bool bExtractSkeleton = false);
	
};


void extractSkeletonFromDiagonals(c_polygon& initPolygon, vector<c_diagonal>& initCuts,vector<c_polygon*>& finalPolygonPieces, SE2d& se, bool bExtractSkeleton=false);

void iterativeDecompose(c_polygon& initPolygon, vector<c_diagonal>& initCuts,double tau, vector<c_polygon*>& finalPolygonPieces,
		vector<c_diagonal>& finalAllCuts, SE2d& se, bool bExtractSkeleton);

void decomposeMoreTimes(c_polygon& initPolygon, vector<c_diagonal>& initCuts,double tau, vector<c_polygon*>& finalPolygonPieces,
		vector<c_diagonal>& finalAllCuts, SE2d& se, bool bExtractSkeleton = false, int iterateTimes=1);


void reInitializePolygon(c_polygon& polygon);

void updateMINMAXConcavity(c_dude& dude2d);

#endif

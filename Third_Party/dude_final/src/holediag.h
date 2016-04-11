#ifndef _HOLEDIAG_H_
#define _HOLEDIAG_H_

#include "polygon.h"
#include "diagonal2.h"
#include "polyline.h"
#include "Point.h"
#include <map>
#include <vector>

using namespace std;
//static uint TMPHOLEFLAG = 99999999;

struct HoleDiagonal{
	ply_vertex* startPnt;
	ply_vertex* endPnt;
	float cutConcavity;
	HoleDiagonal* initParent;//keep the initial linking relationship
	HoleDiagonal* tempParent;//use for some temporary use

	vector<HoleDiagonal*> childHoleDiags;//kee the initial linking relationship
	vector<HoleDiagonal*> tmpChildHoleDiags;//use for some temporary use

	HoleDiagonal(){
		startPnt = NULL;
		endPnt = NULL;
		cutConcavity = 0.0;
		initParent = NULL;
		tempParent = NULL;
	}
	HoleDiagonal(ply_vertex* s,ply_vertex* e)
	{
		startPnt = s;
		endPnt = e;
		cutConcavity = 0.0;
		initParent = NULL;
		tempParent = NULL;
	}
	void clear(){
		startPnt = NULL;
		endPnt = NULL;
		cutConcavity = 0.0;
		initParent = NULL;
		tempParent = NULL;
		for(vector<HoleDiagonal*>::iterator dit = childHoleDiags.begin(); dit != childHoleDiags.end(); ++dit)
		{
			HoleDiagonal* child = *dit;
			child->clear();
		}
	}
	~HoleDiagonal(){
		clear();
		for(vector<HoleDiagonal*>::iterator dit = childHoleDiags.begin(); dit != childHoleDiags.end(); ++dit)
		{
			HoleDiagonal* child = *dit;
			delete child;
		}
	}
};



void insertToRootTree(HoleDiagonal* root,HoleDiagonal* tmpHole);//ply_vertex* tmpStPnt,ply_vertex* tmpEndPnt);

double determineConcavityForConvexHole(c_ply& hole,ply_vertex*& m1,ply_vertex*& m2, ply_vertex*& fV);

float distTwoPnts(const Point2d& s, const Point2d& t);


//reset all the points' concavity except the medial axis end points
void resetConcavityExcludeAxis(c_ply& hole,ply_vertex* m1,ply_vertex* m2);


void collectFeatPntBetweenCuts(c_ply& hole,vector<ply_vertex*>& cuts,list<ply_vertex*>& tmpHolePMs,list<ply_vertex*>& tmpHoleFeatPnts,double tau, vector<HoleDiagonal*>& holeDiags, bool reducePMs=false);

// check if both of two end points of this diagonal is in hole and they are approximated by a single line
bool isHoleDiagonalValid(const ply_vertex* v1, const ply_vertex* v2);

bool checkCutSkinny(const ply_vertex* v1,const ply_vertex* v2, double tau);

#endif

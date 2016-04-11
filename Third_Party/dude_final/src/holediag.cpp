#include "holediag.h"
#include "polygon.h"
#include "diagonal2.h"
#include "dp_approx.h"
#include "intersection.h"
#include "polyline.h"
#include "dp_approx.h"
#include "dude_util.h"
#include <map>
#include <vector>
#include <algorithm>

using namespace std;



void insertToRootTree(HoleDiagonal* root,HoleDiagonal* tmpHole)//ply_vertex* tmpStPnt,ply_vertex* tmpEndPnt)
{
	ply_vertex* rootStPnt = root->startPnt;
	ply_vertex* rootEndPnt = root->endPnt;

	//use the vertex's flag to determine the parent-child relationship
	int rootFlagSt = rootStPnt->getExtra().flag;
	int rootFlagEnd = rootEndPnt->getExtra().flag;
	int ownFlagSt = tmpHole->startPnt->getExtra().flag;
	int ownFlagEnd = tmpHole->endPnt->getExtra().flag;
	bool update_myself = false;
	bool add_to_myself = true;
	vector<HoleDiagonal*>::iterator holeIt = root->childHoleDiags.begin();
	for (;holeIt!=root->childHoleDiags.end();++holeIt)
	{
		HoleDiagonal* kid = *holeIt;
		int kid_flagSt = kid->startPnt->getExtra().flag;
		int kid_flagEnd = kid->endPnt->getExtra().flag;
		//no overlapping, just skip
		if(kid_flagEnd<=ownFlagSt||kid_flagSt>ownFlagEnd)
			continue;
		if(kid_flagSt<=ownFlagSt && kid_flagEnd >=ownFlagEnd)
		{
			//insert to the 
			insertToRootTree(kid,tmpHole);
			add_to_myself = false;
			break;
		}
		else if(ownFlagSt<=kid_flagSt && ownFlagEnd>=kid_flagEnd)
		{
			insertToRootTree(tmpHole,kid);
			update_myself = true;
			//ensure whether needs a "break"
		}
//		else
//			assert(false);
	}
	if(update_myself){
		vector<HoleDiagonal*> tmps;
		tmps.swap(root->childHoleDiags);
		vector<HoleDiagonal*>::iterator kit = tmps.begin();
		for (;kit!=tmps.end();++kit)
		{
			HoleDiagonal* kid = *kit;
			if(kid->initParent==root)
				root->childHoleDiags.push_back(kid);
		}
	}
	if(add_to_myself){
		tmpHole->initParent = root;
		tmpHole->tempParent = root;
		root->childHoleDiags.push_back(tmpHole);
	}
}


void getMidPoint(HoleDiagonal* diag,float& x,float& y)
{
	float rootx1 = diag->startPnt->getPos()[0];
	float rootx2 = diag->endPnt->getPos()[0];
	float rooty1 = diag->startPnt->getPos()[1];
	float rooty2 = diag->endPnt->getPos()[1];
	x = (rootx1+rootx2)/2;
	y = (rooty1+rooty2)/2;
}
//
double findPolesForCutpoly(vector<ply_vertex*>& polygon,ply_vertex*& m1,ply_vertex*& m2)
{
	double maxDist = 0.0,tmpDist=0.0;
	vector<ply_vertex*>::iterator vite = polygon.begin();
	vector<ply_vertex*>::iterator tvite = polygon.begin();
	for (;vite!=polygon.end();++vite)
	{
		ply_vertex* curPnt = *vite;
		tvite = polygon.begin();
		for(;tvite!=polygon.end();++tvite)
		{
			ply_vertex* extraPnt = *tvite;
			if(extraPnt==curPnt)
				continue;
			tmpDist = distTwoPnts(curPnt->getPos(),extraPnt->getPos());
			if(tmpDist>maxDist)
			{
				maxDist = tmpDist;
				m1 = curPnt; m2 = extraPnt;
			}
		}
	}
	return maxDist;
}

float distTwoPnts(const Point2d& s, const Point2d& t)
{
	return sqrtf((s[0]-t[0])*(s[0]-t[0]) + (s[1]-t[1])*(s[1]-t[1]));
}
//check whether the vertex list from p1 to p2 will contain targetV
bool containBetween2V(ply_vertex* p1, ply_vertex*p2, ply_vertex* targetV)
{
	ply_vertex* pSearch = p1->getNext();
	while(pSearch != p2){
		if(pSearch == targetV)
			return true;
		pSearch = pSearch->getNext();
	}
	return false;
}
double determineConcavityForConvexHole(c_ply& hole,ply_vertex*& m1,ply_vertex*& m2, ply_vertex*& fV)
{
	//for each point, find its farthest extra point
	ply_vertex* curPnt = hole.getHead();
	do 
	{
		float tmpConcavity= 0.0;
		ply_vertex* maxDistV = NULL;
		ply_vertex* extraPnt = curPnt->getNext();
		do 
		{
			float tmpDist = distTwoPnts(curPnt->getPos(),extraPnt->getPos());
			if(tmpDist > tmpConcavity)
			{
				tmpConcavity = tmpDist;
				maxDistV = extraPnt;
			}
			extraPnt = extraPnt->getNext();
		} while (extraPnt!=curPnt);

		curPnt->getExtra().concavity = tmpConcavity;
		curPnt->getExtra().other_v = maxDistV;
		//cout<<"current concavity: "<<curPnt->getExtra().concavity<<endl;
		
		curPnt = curPnt->getNext();
	} while (curPnt!=hole.getHead());

	//traverse all the points and get the two points having largest concavity
	m1 = hole.getHead();
	double maxConcavity = m1->getExtra().concavity;
	curPnt = hole.getHead()->getNext();
	do 
	{
		if(curPnt->getExtra().concavity > maxConcavity)
		{
			m1 = curPnt;
			maxConcavity = curPnt->getExtra().concavity;
		}
		curPnt = curPnt->getNext();
	} while (curPnt!=hole.getHead());

	m2 = m1->getExtra().other_v;
	m2->getExtra().concavity = m1->getExtra().concavity;

	//traverse all the points and compute its distance to the medial axis
	ply_vertex* furthestV = m1;
	float maxRectDist = -1;
	curPnt = hole.getHead();
	do{
		float tmpDist = distToSeg(m1->getPos(), m2->getPos(), curPnt->getPos());
		if(maxRectDist < tmpDist){
			maxRectDist = tmpDist;
			furthestV = curPnt;
		}
		curPnt = curPnt->getNext();
	}while(curPnt != hole.getHead());

//	//find a vertex that can create a vector nearly perpendicular to (m1-m2)
//	ply_vertex* pSearchStart = m1;
//	ply_vertex* pSearchEnd = m2;
//	if(containBetween2V(m1, m2, furthestV)){
//		pSearchStart = m2;
//		pSearchEnd = m1;
//	}
//	Vector2d m12(m2->getPos()[0] - m1->getPos()[0], m2->getPos()[1] - m1->getPos()[1]);
//	Vector2d m12Norm = m12.normalize();
//	double MinDotProduct = FLT_MAX;
//	ply_vertex* extraVForFurthestV = NULL;
//	ply_vertex* pSearchCur = pSearchStart;
//	while(pSearchCur != pSearchEnd)
//	{
//		assert(pSearchCur != furthestV);
//		Vector2d furVec(pSearchCur->getPos()[0] - furthestV->getPos()[0], pSearchCur->getPos()[1] - furthestV->getPos()[1]);
//		Vector2d tmpNorm  = furVec.normalize();
//		double tmpProduct = abs(tmpNorm * m12Norm);
//		if(tmpProduct < MinDotProduct){
//			MinDotProduct = tmpProduct;
//			extraVForFurthestV = pSearchCur;
//		}
//		pSearchCur = pSearchCur->getNext();
//	}
//
//	maxRectDist = distTwoPnts(furthestV->getPos(), extraVForFurthestV->getPos());



	//reset all the other points' concavity except the medial axis end points
	resetConcavityExcludeAxis(hole,m1,m2);

	fV = furthestV;
	return std::min(maxConcavity, 2.0*maxRectDist);

}
void resetConcavityExcludeAxis(c_ply& hole,ply_vertex* m1,ply_vertex* m2)
{
	ply_vertex* pCursor = hole.getHead();
	do 
	{
		if(pCursor!=m1&&pCursor!=m2)
		{
			pCursor->getExtra().concavity = 0;
			pCursor->getExtra().other_v = NULL;
		}
		pCursor = pCursor->getNext();
	} while (pCursor!=hole.getHead());
}
HoleDiagonal* getHoleDiag(ply_vertex* v1, ply_vertex* v2, vector<HoleDiagonal*>& holeDiags)
{
	for(vector<HoleDiagonal*>::iterator dit = holeDiags.begin(); dit != holeDiags.end(); ++dit)
	{
		HoleDiagonal* tdig = *dit;
		if( (tdig->startPnt == v1 && tdig->endPnt == v2) ||(tdig->startPnt == v2 && tdig->endPnt)){
			return tdig;
		}
	}
	return NULL;
}
void collectFeatPntBetweenCuts(c_ply& hole,vector<ply_vertex*>& cuts,list<ply_vertex*>& tmpHolePMs,list<ply_vertex*>& tmpHoleFeatPnts,double tau, vector<HoleDiagonal*>& holeDiags, bool reducePMs)
{
	uint tflag = ply_vertex_extra::getFlagID();
	for(vector<ply_vertex*>::const_iterator dit = cuts.begin(); dit != cuts.end(); ++dit){
		ply_vertex* ct = *dit;
		ct->getExtra().flag = tflag;
	}

	ply_vertex * startV = cuts.front();
	ply_vertex * pv = startV;
	do{
		list<ply_vertex*> tmpLine;
		tmpLine.push_back(pv);
		do{
			pv = pv->getNext();
			tmpLine.push_back(pv);

		}while(pv->getExtra().flag != tflag);

		list<ply_vertex*> tmpApproxlist;

		dp_approximate(tmpLine.front(), tmpLine.back(),4*tau/5, tmpApproxlist);
		for(list<ply_vertex*>::iterator vit = tmpApproxlist.begin(); vit != tmpApproxlist.end(); ++vit){
			ply_vertex* tmpv = *vit;
			if(tmpv->isReflex())
			{
				double toDiagDist = distToSeg(tmpLine.front()->getPos(), tmpLine.back()->getPos(), tmpv->getPos());
				tmpv->getExtra().concavity = min(tmpLine.front()->getExtra().holeCutConcavity, tmpLine.back()->getExtra().holeCutConcavity) + toDiagDist;
				if(reducePMs && (toDiagDist > tau))
					tmpHolePMs.push_back(tmpv);//this is a hole pocket minimum vertex
				else if(tmpv->getExtra().concavity > tau)
					tmpHolePMs.push_back(tmpv);//this is a hole pocket minimum vertex
			}
			//tmpv->getExtra().concavity_hole = &hole;
			tmpHoleFeatPnts.push_back(tmpv);
		}

	}while(pv != startV);
}
bool isHoleDiagonalValid(const ply_vertex* v1, const ply_vertex* v2)
{
	if(v1->getExtra().getConcavity_hole()==NULL && v2->getExtra().getConcavity_hole()==NULL)
		return true;

	const ply_vertex* helpV = NULL;
	const ply_vertex* minimumV = NULL;
	const ply_vertex* minOtherV = NULL;
	if(v1->getExtra().getConcavity_hole()!=NULL && v1->getExtra().other_v!=NULL)
	{
		helpV = v1->getExtra().other_v;
		minimumV = v1;
		minOtherV = v2;
	}
	else if(v2->getExtra().getConcavity_hole()!=NULL && v2->getExtra().other_v!=NULL)
	{
		helpV = v2->getExtra().other_v;
		minimumV = v2;
		minOtherV = v1;
	}

	if(helpV == NULL)
		return true;
	const ply_vertex* v1pre = helpV->getPre()->extra.other_v;
	const ply_vertex* v1next = helpV->getNext()->extra.other_v;


	const ply_vertex*  pcur = minimumV;
	do{
		pcur = pcur->getPre();
		if(pcur == minOtherV)
			return false;
	}while(pcur != v1pre && pcur!=minimumV);

	pcur = minimumV;
	do{
		pcur = pcur->getNext();
		if(pcur == minOtherV)
			return false;
	}while(pcur != v1next && pcur!=minimumV);


	return true;
}
bool checkCutSkinny(const ply_vertex* v1,const ply_vertex* v2, double tau)
{
	if(v1==v2) return true;

	double accumulatedLength = 0.0;
	const ply_vertex* pcur = v1;
	const ply_vertex* pnext = pcur->getNext();
	while(accumulatedLength < 2*tau){
		accumulatedLength += distance2Point(pcur->getPos(), pnext->getPos());
		if(pnext == v2)
			return true;

		pcur = pnext;
		pnext = pnext->getNext();
	}

	accumulatedLength = 0.0;
	pcur = v2;
	pnext = pcur->getNext();
	while(accumulatedLength < 3*tau){
		accumulatedLength += distance2Point(pcur->getPos(), pnext->getPos());
		if(pnext == v1)
			return true;

		pcur = pnext;
		pnext = pnext->getNext();
	}

	return false;
}



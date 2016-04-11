#include "holecutconcavity.h"
#include "polygon.h"
#include <map>
#include <vector>
#include <limits>
#include <algorithm>
#include "intersection.h"
#include "dp_approx.h"
#include "Point.h"
using namespace std;


c_plyline list2polyline( const list<ply_vertex*>& vlist )
{	
	typedef list<ply_vertex*>::const_iterator IT;
	c_plyline ply;
	ply.beginPoly();
	for(IT i=vlist.begin();i!=vlist.end();i++){
		const ply_vertex* v=*i;
		const Point2d& pos=v->getPos();
		ply.addVertex(pos[0],pos[1]);
	}

	ply.endPoly();

	return ply;
}

void collectCutPoints(const vector<c_diagonal>&cuts,vector<ply_vertex*>& collectList)
{
	collectList.clear();
	vector<c_diagonal>::const_iterator dite = cuts.begin();
	for (;dite!=cuts.end();++dite)
	{
		ply_vertex* s = dite->getV1();
		ply_vertex* e = dite->getV2();
		collectList.push_back(s);
		collectList.push_back(e);
	}
}

bool checkCutPoint(vector<ply_vertex*>& collectList,ply_vertex* tmpV)
{
	vector<ply_vertex*>::iterator vite = collectList.begin();
	for (;vite!=collectList.end();++vite)
	{
		if(tmpV==(*vite))
			return true;
	}
	return false;
}
bool checkListContainPnt(list<ply_vertex*>& vlist,ply_vertex* tmpV)
{
	list<ply_vertex*>::iterator vite = vlist.begin();
	for (;vite!=vlist.end();++vite)
	{
		if(tmpV==(*vite))
			return true;
	}
	return false;
}

int computeHierarchyDepth(HoleDiagonal* root)
{
	if(root->childHoleDiags.size() == 0)
		return 0;
	int depth = 0;
	for(vector<HoleDiagonal*>::iterator dit = root->childHoleDiags.begin(); dit != root->childHoleDiags.end(); ++dit)
	{
		if(*dit == NULL)
			continue;
		int tmpDepth = computeHierarchyDepth(*dit) + 1;
		if(tmpDepth > depth)
			depth = tmpDepth;
	}
	return depth;
}
void splitHoleByFlagUsingDiag(ply_vertex* start,ply_vertex* end)
{
	//mark the points from startPoint,start point has the smallest flag id
	int count1=1;
	ply_vertex* sp = start;//hole.getHead();
	do
	{
		sp->getExtra().flag = count1++;
		sp = sp->getNext();
	} while (sp!=end);
	int count2 = 1;
	sp = start;
	do{
		sp->getExtra().flag = count2++;
		sp = sp->getPre();
	}while(sp!=end);
	end->getExtra().flag = std::max(count1 + 1,count2 + 1);
}
void buildHoleDiagonalHierarchy(HoleDiagonal* root, const vector<c_diagonal>& holecuts)
{
	ply_vertex* s = root->startPnt;
	ply_vertex* e = root->endPnt;
	ply_vertex* p = s;
	int count = 1;
	while(p != e){
		p->getExtra().flag = count++;
		p = p->getNext();
	}
	e->getExtra().flag = count;

	//insert all the cuts into  the root's tree
	for(vector<c_diagonal>::const_iterator j=holecuts.begin();j!=holecuts.end();j++)
	{
		const c_diagonal& dia=*j;
		//always make sure start point has smaller flag id than end point
		ply_vertex* stPnt = dia.getV1()->getExtra().flag < dia.getV2()->getExtra().flag ? dia.getV1(): dia.getV2();
		ply_vertex* endPnt = dia.getV1()->getExtra().flag < dia.getV2()->getExtra().flag ? dia.getV2(): dia.getV1();
		if((stPnt==root->startPnt)&&(endPnt==root->endPnt))//if this is the root cut, continue;
			continue;
		HoleDiagonal* tmpHole = new HoleDiagonal(stPnt,endPnt);

		insertToRootTree(root,tmpHole);//stPnt,endPnt);
	}
}

void splitCutsToTwo(ply_vertex* s,ply_vertex*e,const vector<c_diagonal>& holecuts, vector<c_diagonal>& left, vector<c_diagonal>& right)
{
	for(vector<c_diagonal>::const_iterator dit = holecuts.begin(); dit != holecuts.end(); ++dit)
	{
		const c_diagonal& diag = *dit;
		if( (diag.getV1() == s && diag.getV2() == e) || (diag.getV1() == e && diag.getV2() == s) )
			continue;

		if(diag.getV1()->getExtra().flag <= e->getExtra().flag && diag.getV2()->getExtra().flag <= e->getExtra().flag){
			left.push_back(diag);
		}
		else{
			right.push_back(diag);
		}
	}
}

//this function returns a good diagonal root from the cuts of hole
HoleDiagonal* getCutDiagRoot(const vector<c_diagonal>& holecuts)
{
	if(holecuts.size() == 0)
		return NULL;
	if(holecuts.size() == 1)
		return (new HoleDiagonal(holecuts.front().getV1(), holecuts.front().getV2()));

	//1.mark all the hole cuts' end points
	for(vector<c_diagonal>::const_iterator dit = holecuts.begin(); dit != holecuts.end(); ++dit)
	{
		const c_diagonal& dia = *dit;
		dia.getV1()->getExtra().isHoleDiag = true;
		dia.getV2()->getExtra().isHoleDiag = true;
	}

	int tmpBestDepth = 9999;
	HoleDiagonal* bestRoot = NULL;
	//loop over all the cuts and use them as root candidate
	for(vector<c_diagonal>::const_iterator dit = holecuts.begin(); dit != holecuts.end(); ++dit)
	{
		const c_diagonal& dia = *dit;

		 ply_vertex* s = dia.getV1();
		 ply_vertex* t = dia.getV2();
		 ply_vertex* p = s;
		int count = 1;
		do{
			p->getExtra().flag = count++;
			p = p->getNext();
		}while(p != s);

		vector<c_diagonal> left;
		vector<c_diagonal> right;
		splitCutsToTwo(s, t, holecuts, left, right);

		HoleDiagonal* root1 = new HoleDiagonal(s, t);
		buildHoleDiagonalHierarchy(root1, left);
		HoleDiagonal* root2 = new HoleDiagonal(t ,s);
		buildHoleDiagonalHierarchy(root2, right);

		root1->childHoleDiags.insert(root1->childHoleDiags.end(), root2->childHoleDiags.begin(), root2->childHoleDiags.end());
		int tmpD = computeHierarchyDepth(root1);
		if(tmpD < tmpBestDepth){
			tmpBestDepth = tmpD;
			if(bestRoot != NULL)
				delete bestRoot;
			bestRoot = root1;
		}
		else{
			delete root1;
		}
	}

	return bestRoot;
}
bool checkParentChild(HoleDiagonal* parent, HoleDiagonal* child)
{
	for(vector<HoleDiagonal*>::iterator hit = parent->childHoleDiags.begin(); hit != parent->childHoleDiags.end(); ++hit)
	{
		if((*hit)==child)
			return true;
	}

	return false;
}
void updateChildParentRelation(HoleDiagonal* root)
{
	if(root->childHoleDiags.size() == 0)
		return;

	double a1 = (root->startPnt->getPos()[0] + root->endPnt->getPos()[0]) / 2;
	double b1 = (root->startPnt->getPos()[1] + root->endPnt->getPos()[1]) / 2;
	for(vector<HoleDiagonal*>::iterator dit = root->childHoleDiags.begin(); dit != root->childHoleDiags.end(); ++dit)
	{
		HoleDiagonal* tdig = *dit;
		tdig->tempParent=  root;
		tdig->initParent = root;

		double a2 = (tdig->startPnt->getPos()[0] + tdig->endPnt->getPos()[0]) / 2;
		double b2 = (tdig->startPnt->getPos()[1] + tdig->endPnt->getPos()[1]) / 2;
		tdig->cutConcavity = root->cutConcavity + distance2D(a1, b1, a2, b2);
		tdig->startPnt->getExtra().holeCutConcavity = tdig->cutConcavity;
		tdig->endPnt->getExtra().holeCutConcavity = tdig->cutConcavity;

		updateChildParentRelation(tdig);
	}
}
void constructHierarchyConcavity(HoleDiagonal* root)
{
	root->cutConcavity = 0;
	root->startPnt->getExtra().holeCutConcavity = 0;
	root->endPnt->getExtra().holeCutConcavity = 0;

	updateChildParentRelation(root);

}
void hierarchyToList(HoleDiagonal* root, vector<HoleDiagonal*>& holeDiagVec)
{
	holeDiagVec.push_back(root);
	for(vector<HoleDiagonal*>::iterator dit = root->childHoleDiags.begin(); dit != root->childHoleDiags.end(); ++dit)
	{
		HoleDiagonal* tdig = *dit;
		hierarchyToList(tdig, holeDiagVec);
	}
}





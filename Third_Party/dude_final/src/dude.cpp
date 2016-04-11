#include "dude.h"
#include "intersection.h"
#include "cdt2.h"
#include "knapsack.h"
#include "chull.h"
#include "eigen.h"
#include "poly_approx.h"
#include "holediag.h"
#include "diagonal2.h"
#include "dp_approx.h"
#include "dude_cut.h"
#include "holecutconcavity.h"
#include "dude_util.h"
#include "intersection.h"
#include <set>
#include <map>
#include <vector>

#include "draw_decoration.h"

using namespace std;

extern Draw_Decoration draw_decoration;

#define INPOCKETFLAG 999999
int HOLEPARAMETER = 1;
int HOLELINEAPPROXPARA = 1;
#define HOLECONCAVITYRATIO 1 //0.75
#define SMALLZERO 1e-8
#define REDUCEPOCKETMINIMUM true



void c_dude::build(c_polygon& P, double tau, bool isInitPolygon)
{
//    int psize=P.getSize();
//    for(int i=0;i<psize;i++){
//     if(P[i]->getExtra().concavity_bpc) cout<<P[i]->getExtra().concavity_bpc<<endl;
//    }


	//set tolerance
	m_tau=tau;

    //build BPC
    buildBPC(P);

	//by Guilin, mark all the point's parent ply
	//markParentPly(&P);
    
	if(m_bpcs.empty() && P.size() == 1)
	{
		P.bCnveXEnough = true;
		return;
	}

	/**************collect the feature points******************/
	//collect the feature points inside the points
    collectPMs(m_bpcs);

	collectConvexFeatPnts(P,convexFeatPnts);

	//collect the features points on the external-boundary
	collectOutBoundFeatPnts();

	//cout<<"total hole featured point number is "<<this->holeFeatPnts.size()<<endl;

	//collect the features points 
	collectTotalFeatPnts();
	/**********************************************************/
	//clear the previous diagonals
	clearVerticesDiag(P);

    create_simplified_and_reduced(P);

    if(m_simpilifed_P.getSize()==0)
    {
    	P.bCnveXEnough = true;
    	return;
    }

    //do it again
   // markParentPly(&P);

   // markParentPly(&m_simpilifed_P);

    //build CDT diagonals
    c_cdt cdt_simplified;
    cdt_simplified.build(*this, m_simpilifed_P,P);

 //   markParentPly(&m_reduced_P);
    c_cdt cdt_reduced;
    cdt_reduced.build(*this, m_reduced_P,P);

    //build cutsets
    //buildCutSets(m_PMs);

	//build cutsets(also for the hole points)
	combineMinimumPnts(m_PMs,/*holeAxisPnts*/hole_PMs,combine_PMs);
	buildCutSets(combine_PMs);

    keepKBest(combine_PMs, m_cuts);


    if(isInitPolygon)
    {
    	resolveIntersectedCuts(P);
    }

//	//
//	accumulatedCuts.push_back(m_cuts);
//
//	//
//	allSimplifiedPolys.push_back(m_simpilifed_P);
}

void c_dude::combineMinimumPnts(list<ply_vertex*>& extraPMs,list<ply_vertex*>& holePMs,list<ply_vertex*>& totalPMs)
{
	typedef list<ply_vertex*>::iterator PIT;
	set<ply_vertex*> PMset;
	for (PIT pit = extraPMs.begin();pit!=extraPMs.end();++pit)
	{
		PMset.insert(*pit);
	}
	for (PIT pit = holePMs.begin();pit!=holePMs.end();++pit)
	{
		PMset.insert(*pit);
	}
	
	totalPMs.insert(totalPMs.end(),PMset.begin(),PMset.end());
}

//check whether the vertex is contained in the 
int c_dude::checkStartPoint(ply_vertex* curVert)
{
	int resIdx = 0;
	vector<ply_vertex*>::iterator vit = bpcStartPnts.begin();
	for (;vit!=bpcStartPnts.end();++vit,++resIdx)
	{
		if(curVert == (*vit))
			return resIdx;
	}
	return -1;
}
int c_dude::checkEndPoint(ply_vertex* curVert)
{
	int resIdx = 0;
	vector<ply_vertex*>::iterator vit = bpcEndPnts.begin();
	for (;vit!=bpcEndPnts.end();++vit,++resIdx)
	{
		if(curVert == (*vit))
			return resIdx;
	}
	return -1;
}

void c_dude::collectOutConvexFeatPnts(c_ply& plyOut, list<ply_vertex*>& tmpconvexFeatPnts)
{
	//get two vertices having maximum concavity
	ply_vertex* m1,*m2;
	ply_vertex* fv;
	double maxConcavity = determineConcavityForConvexHole(plyOut,m1,m2,fv);
	m1->getExtra().concavity = 0;
	m2->getExtra().concavity = 0;

	list<ply_vertex*> vList1,vList2,approx1,approx2;
	map<ply_vertex*,ply_vertex*> vmap1, vmap2;

	//fill the two vertex list
	ply_vertex* curPnt = m1;
	do{
		vList1.push_back(curPnt);
		curPnt = curPnt->getNext();
	}while(curPnt != m2);
	vList1.push_back(curPnt);
	assert(curPnt == m2);

	do{
		vList2.push_back(curPnt);
		curPnt = curPnt->getNext();
	}while(curPnt!=m1);
	vList2.push_back(curPnt);
	assert(curPnt == m1);

	//approximate these two vertex lists
	c_plyline line1 = bpc2polyline(vList1);

	approximateMedthod(line1,m_tau/*/HOLELINEAPPROXPARA*/,approx1);


	c_plyline line2 = bpc2polyline(vList2);

	approximateMedthod(line2,m_tau/*/HOLELINEAPPROXPARA*/,approx2);


	//map points from plyline to initial ply
	{
		ply_vertex *j=line1.getHead();
		for(list<ply_vertex*>::iterator i=vList1.begin();i!=vList1.end();++i,j=j->getNext()){
			vmap1[j]=*i;
		}
	}
			//map points from plyline to initial ply
			{
				ply_vertex *j=line2.getHead();
				for(list<ply_vertex*>::iterator i=vList2.begin();i!=vList2.end();++i,j=j->getNext()){
					vmap2[j]=*i;
				}
			}
			assert(vmap1[line1.getHead()] == m1 && vmap1[line1.getTail()] == m2);
			assert(vmap2[line2.getHead()] == m2 && vmap2[line2.getTail()] == m1);

			tmpconvexFeatPnts.push_back(m1);tmpconvexFeatPnts.push_back(m2);

			//featPnts.push_back(m1); featPnts.push_back(m2);
			//holeAxisPnts.push_back(m1);holeAxisPnts.push_back(m2);
			int approxVNum = 2;

			//put the approximate points into the approximate vertex list
			for (list<ply_vertex*>::iterator vlIte=approx1.begin();vlIte!=approx1.end();++vlIte)
			{
				ply_vertex* tmpPnt = vmap1[*vlIte];
				//assert(tmpPnt != m1->getPre() && tmpPnt != m2->getPre() && tmpPnt != m1->getNext() && tmpPnt != m2->getNext());
				if(tmpPnt==m1||tmpPnt==m2)
					continue;

				tmpconvexFeatPnts.push_back(tmpPnt);

				approxVNum++;
			}
			for (list<ply_vertex*>::iterator vlIte=approx2.begin();vlIte!=approx2.end();++vlIte)
			{
				ply_vertex* tmpPnt = vmap2[*vlIte];
				//assert(tmpPnt != m1->getPre() && tmpPnt != m2->getPre() && tmpPnt != m1->getNext() && tmpPnt != m2->getNext());
				if(tmpPnt == m1 || tmpPnt == m2)
					continue;

				tmpconvexFeatPnts.push_back(tmpPnt);

				approxVNum++;
			}
}
void c_dude::collectConvexFeatPnts(c_polygon& P,list<ply_vertex*>& boundFeatPnts)
{
	bpcStartPnts.clear();
	bpcEndPnts.clear();
	typedef list<c_BPC*>::iterator BIT;
	//put the start points and end points to the vector
	for (BIT cit = m_bpcs.begin();cit!=m_bpcs.end();++cit)
	{
		c_BPC * tmpbpc = *cit;
		bpcStartPnts.push_back(tmpbpc->getSource1());
		bpcEndPnts.push_back(tmpbpc->getSource2());
	}//

	//mark all the points inside the m_bcps
	uint tempFlag = getDudeFlag();//INPOCKETFLAG;
	for (BIT cit = m_bpcs.begin();cit!=m_bpcs.end();++cit)
	{
		c_BPC * tmpbpc = *cit;
		ply_vertex * tmpVert = tmpbpc->getSource1();
		tmpVert = tmpVert->getNext();
		while (tmpVert!=tmpbpc->getSource2())
		{
			tmpVert->getExtra().flag = tempFlag;
			tmpVert = tmpVert->getNext();
		}

	}

	//get the unmarked point list and also get the start point of the line
	//since the function approximate() only work for a plyline, 
	//so, here, select a end point of a pocket as the  start point of the plyline
	//and its start point as the end point of the plyline.
	/**************************************************************/
	int totalPlySize = P.size();
	for(c_polygon::iterator pit=P.begin();pit!=P.end();pit++)
	{
		c_ply& extrBound = *pit;
		if(extrBound.getType() == c_ply::PIN)
			continue;

		if(m_bpcs.size() == 0 && totalPlySize != 1){
			collectOutConvexFeatPnts(extrBound, boundFeatPnts);
			//cout<<"found boundaryFeatPnts number "<<boundFeatPnts.size()<<endl;
			//cout<<"discoverd!"<<endl;
			return;
		}


		list<ply_vertex*> vList;
		list<ply_vertex*> vList1,vList2;

		bool splitTag = false;
		ply_vertex* tVert = extrBound.getHead();
		do 
		{
			//if the point is NOT inside a pocket
			if(tVert->getExtra().flag!=tempFlag)
			{
				if (!splitTag)
				{
					int vidx = checkStartPoint(tVert);
					if(vidx!=-1)//find a start point of a pocket, use as the splitting point of the list
					{
						splitTag = true;
					}
					vList1.push_back(tVert);
				}
				else
				{
					vList2.push_back(tVert);
				}
			}
			tVert = tVert->getNext();
		} while (tVert!=extrBound.getHead());

		int vertSize = extrBound.getSize();
		if((vList1.size()+vList2.size()==(uint)vertSize)&&(extrBound.getType()==c_ply::POUT))//the ply is convex
		{
			//cout<<"find a convex ply"<<endl;
			list<ply_vertex*> convList1,convList2;
			ply_vertex* stPnt = extrBound.getHead();
			ply_vertex* endPnt = stPnt;
			int vsize = extrBound.getSize();
			for(int i=0;i<vsize/2;++i)
			{
				endPnt = endPnt->getNext();
			}
			ply_vertex* curPnt = stPnt;
			convList1.push_back(curPnt);
			do 
			{
				curPnt = curPnt->getNext();
				convList1.push_back(curPnt);
			} while (curPnt!=endPnt);
			curPnt = endPnt;
			convList2.push_back(curPnt);
			do 
			{
				curPnt = curPnt->getNext();
				convList2.push_back(curPnt);
			} while (curPnt!=stPnt);
			collectListFeatPnts(convList1,boundFeatPnts);
			collectListFeatPnts(convList2,boundFeatPnts);
			return;
			//continue;
		}

		//put two lists together
		list<ply_vertex*>::iterator vit = vList2.begin();
		for (;vit!=vList2.end();++vit)
		{
			vList.push_back(*vit);
		}
		for (vit=vList1.begin();vit!=vList1.end();++vit)
		{
			vList.push_back(*vit);
		}

		//split the vList into several parts
		list<ply_vertex*>::iterator curIt = vList.begin();
		while (curIt!=vList.end())
		{
			//use the start points and end points to split the line into several parts, and approximate each part individually
			list<ply_vertex*> tempList;
			int tmpStartIdx = checkStartPoint(*curIt);
			int tmpEndIdx = checkEndPoint(*curIt);
			

			if (tmpStartIdx!=-1)
			{
				++curIt;
			}
			if(curIt==vList.end())
				break;
			while ((curIt!=vList.end())&&(checkStartPoint(*curIt)==-1))
			{
				tempList.push_back(*curIt);
				++curIt;
			}
			//approximate the current plyline
			if (tempList.size()>2)
			{
				list<ply_vertex*> approx;
				map<ply_vertex*,ply_vertex*> vmap, vmap2;
				//create a ply_line for this vertex list
				c_plyline tmp = bpc2polyline(tempList);


				approximateMedthod(tmp,m_tau - m_tau/4, approx);


				//map from tmp (store in "tmp") to the original polygon
				{
					ply_vertex *j=tmp.getHead();
					for(list<ply_vertex*>::iterator i=tempList.begin();i!=tempList.end();i++){
						vmap[j]=*i;
						j=j->getNext();
					}
				}
				//mark the approximation
				for(list<ply_vertex*>::iterator ait = approx.begin();
					ait!=approx.end();++ait)
				{
					ply_vertex* v = vmap[*ait];
					assert(v);
					boundFeatPnts.push_back(v);
				}
			}
		}
	}
}
void c_dude::collectListFeatPnts(list<ply_vertex*>& vlist,list<ply_vertex*>& resultVList)
{
	list<ply_vertex*> approx;
	map<ply_vertex*,ply_vertex*> vmap, vmap2;
	c_plyline hullply= bpc2polyline(vlist);

	approximateMedthod(hullply,m_tau-m_tau/4,approx);

	//map from hullply to tmp (store in "hull")
	{
		ply_vertex *j=hullply.getHead();
		for(list<ply_vertex*>::iterator i=vlist.begin();i!=vlist.end();i++){
			vmap2[j]=*i;
			j=j->getNext();
		}
	}
	//mark the approximation
	for(list<ply_vertex*>::iterator ait = approx.begin();
		ait!=approx.end();++ait)
	{
		ply_vertex* v = vmap2[*ait];
		assert(v);

		resultVList.push_back(v);
	}
}
////refine the pocket minimum points
//void c_dude::refinePocketPMs()
//{
//	 typedef list<c_BPC*>::iterator BIT;
//	uint tmpFlag = ply_vertex_extra::getFlagID();
//
//	for(list<ply_vertex*>::iterator pite = totalFeatPnts.begin();pite!=totalFeatPnts.end();++pite)
//	{
//		(*pite)->getExtra().flag = tmpFlag;
//	}
//
//	for(BIT i=m_bpcs.begin();i!=m_bpcs.end();i++)
//	{
//		c_BPC* bpc=*i;
//	}
//
////
////	do{
////
////			//marked as a special vertex
////			if(ptr->getExtra().flag==m_marking_flag)
////			{
////				const Point2d& pt=ptr->getPos();
////				ply_vertex * new_v=to.addVertex(pt[0],pt[1]);
////
////				//build the map between from vertices and to vertices
////				new_v->getExtra().other_v=ptr;
////				ptr->getExtra().other_v=new_v;
////
////				if(isHole)
////				{
////					ptr->getExtra().setConcavity_hole(&from);
////					new_v->getExtra().setConcavity_hole(&to);
////				}
////
////				if(ptr->getExtra().concavity_bpc!=NULL)
////					found_concavity=true;
////			}
////
////	        ptr=ptr->getNext();
////	    }
////	    while(ptr!=from.getHead());
//
//
//
//}

void c_dude::create_simplified_and_reduced(c_polygon& P)
{
	typedef list<c_BPC*>::iterator BIT;
	m_marking_flag=ply_vertex_extra::getFlagID();

	for(list<ply_vertex*>::iterator pite = totalFeatPnts.begin();pite!=totalFeatPnts.end();++pite)
	{
		(*pite)->getExtra().flag=m_marking_flag;
	}
     //Create m_simpilifed_P and m_reduced_P
     for(c_polygon::iterator i=P.begin();i!=P.end();i++){
         c_ply ply(i->getType());
         buildSimpliedPly(*i,ply);
         if(ply.getHead()!=NULL){
			if(ply.getSize()<3)
				continue;
             m_simpilifed_P.push_back(ply);
			 m_reduced_P.push_back(*i);
         }
     }
}

//
// create BPC
//
void c_dude::buildBPC(c_polygon& P)
{
	for(c_polygon::iterator i=P.begin();i!=P.end();){
		if(i->getType()==c_ply::POUT)
		{
		    buildBPC_from_ext(*i);
		    i++;
		}
		else
		{
		    buildBPC_from_hole(*i);
		    if(i->canBeIgnored)
		    	i = P.erase(i);
		    else
		    	i++;
		}
	}
}

void c_dude::buildBPC_from_hole(c_ply& P)
{
	//decompose the hole
    list<c_BPC*> bpcs;
	decompose_hole(P,hole_PMs,holeFeatPnts,bpcs);
}

//list<ply_vertex *> g_hullply;

void c_dude::buildBPC_from_ext(c_ply& P)
{
    if(P.getSize()<=3) return;

	//build convex hull
	list<ply_vertex *> hull;
	hull2d(P.getHead(),P.getHead(),hull);

	typedef list<ply_vertex *>::iterator IT;

#if DEBUG
	cout<<"- Convex Hull:";
	for(IT i=hull.begin();i!=hull.end();i++) cout<<(*i)->getVID()<<", ";
	cout<<endl;
#endif

	//build pockets

	int count=0;
	for(IT i=hull.begin();i!=hull.end();i++)
	{
		IT j=i; j++;
		if(j==hull.end()) j=hull.begin();
		if(*i==*j) continue; //same node...?
		
#if DEBUG
		cout<<"Bridge ("<<(*i)->getVID()<<","<<(*j)->getVID()<<")"<<endl;
#endif

        //build BPCs here
        c_BPC * bpc=new c_BPC();
        //assert(bpc);
        if(bpc->build(*i,*j)==false){
            delete bpc;
            continue;
        }

        decompose_bridge(bpc);

		//build the hierarchy of the bpcs
		bpc->reorganize_kids_f();
		//comment by Guilin

        //compute the concavity of this bpc
        bpc->determineConcavity(m_tau);

        bpc->determine_PM(m_tau);

		count++;

		m_bpcs.push_back(bpc);


        //break;

	}//end for i
}
void c_dude::decompose_hole(c_ply& hole, list<ply_vertex*>& tmpholePMs,list<ply_vertex*>& tmpholeFeatPnts, list<c_BPC *>& bpcs)
{
	//treat the hole as a polygon
    c_polygon polygon;
    hole.reverseType();
    polygon.push_back(hole);
    c_dude dude;
    dude.build(polygon, m_tau * HOLECONCAVITYRATIO, false);
	hole.reverseType();

	//get the final cut for this hole
	const vector<c_diagonal>& holecuts=dude.getFinalCuts();

	if(holecuts.size()==0/*||dude.m_bpcs.size()==0*/)
	{
		//get two vertices having maximum concavity
		ply_vertex* m1,*m2;
		ply_vertex* furthestV;
		double maxConcavity = determineConcavityForConvexHole(hole,m1,m2,furthestV);

		if(maxConcavity <  m_tau)
		{
			hole.canBeIgnored = true;
			return;
		}
		list<ply_vertex*> vList1,vList2,approx1,approx2;
		map<ply_vertex*,ply_vertex*> vmap1, vmap2;

		//fill the two vertex list
		ply_vertex* curPnt = m1;
		do{
			vList1.push_back(curPnt);
			curPnt = curPnt->getNext();
		}while(curPnt != m2);
		vList1.push_back(curPnt);
		assert(curPnt == m2);

		do{
			vList2.push_back(curPnt);
			curPnt = curPnt->getNext();
		}while(curPnt!=m1);
		vList2.push_back(curPnt);
		assert(curPnt == m1);

		//approximate these two vertex lists
		c_plyline line1 = bpc2polyline(vList1);
		//approximate(line1,m_tau/HOLELINEAPPROXPARA,approx1);
		approximateMedthod(line1,m_tau/*/HOLELINEAPPROXPARA*/,approx1);


		c_plyline line2 = bpc2polyline(vList2);
		//approximate(line2,m_tau/HOLELINEAPPROXPARA,approx2);
		approximateMedthod(line2,m_tau/*/HOLELINEAPPROXPARA*/,approx2);


		
		//map points from plyline to initial ply
		{
			ply_vertex *j=line1.getHead();
			for(list<ply_vertex*>::iterator i=vList1.begin();i!=vList1.end();++i,j=j->getNext()){
				vmap1[j]=*i;
			}
		}
		//map points from plyline to initial ply
		{
			ply_vertex *j=line2.getHead();
			for(list<ply_vertex*>::iterator i=vList2.begin();i!=vList2.end();++i,j=j->getNext()){
				vmap2[j]=*i;
			}
		}
		assert(vmap1[line1.getHead()] == m1 && vmap1[line1.getTail()] == m2);
		assert(vmap2[line2.getHead()] == m2 && vmap2[line2.getTail()] == m1);


		tmpholePMs.push_back(m1);tmpholePMs.push_back(m2);
		tmpholeFeatPnts.push_back(m1);tmpholeFeatPnts.push_back(m2);

		int approxVNum = 2;


//		//put the approximate points into the approximate vertex list
//		for (list<ply_vertex*>::iterator vlIte=approx1.begin();vlIte!=approx1.end();++vlIte)
//		{
//			ply_vertex* tmpPnt = vmap1[*vlIte];
//					//assert(tmpPnt != m1->getPre() && tmpPnt != m2->getPre() && tmpPnt != m1->getNext() && tmpPnt != m2->getNext());
//					if(tmpPnt==m1||tmpPnt==m2)
//						continue;
//					//update the concavity for this approximate point
//					tmpPnt->getExtra().concavity = distToSeg(m1->getPos(),m2->getPos(),tmpPnt->getPos());
//
//					if(tmpPnt->getExtra().concavity > m_tau)
//						tmpholePMs.push_back(tmpPnt);
//
//
//					tmpholeFeatPnts.push_back(tmpPnt);
//
//					approxVNum++;
//				}
//				for (list<ply_vertex*>::iterator vlIte=approx2.begin();vlIte!=approx2.end();++vlIte)
//				{
//					ply_vertex* tmpPnt = vmap2[*vlIte];
//					//assert(tmpPnt != m1->getPre() && tmpPnt != m2->getPre() && tmpPnt != m1->getNext() && tmpPnt != m2->getNext());
//					if(tmpPnt == m1 || tmpPnt == m2)
//						continue;
//
//					//update the concavity for this approximate point
//					tmpPnt->getExtra().concavity = distToSeg(m1->getPos(),m2->getPos(),tmpPnt->getPos());
//
//					if(tmpPnt->getExtra().concavity > m_tau)
//						tmpholePMs.push_back(tmpPnt);
//					tmpholeFeatPnts.push_back(tmpPnt);
//
//					approxVNum++;
//				}







		//put the approximate points into the approximate vertex list
		for (list<ply_vertex*>::iterator vlIte=approx1.begin();vlIte!=approx1.end();++vlIte)
		{ 	
			ply_vertex* tmpPnt = vmap1[*vlIte];
			//assert(tmpPnt != m1->getPre() && tmpPnt != m2->getPre() && tmpPnt != m1->getNext() && tmpPnt != m2->getNext());
			if(tmpPnt==m1||tmpPnt==m2)
				continue;
			//update the concavity for this approximate point
			tmpPnt->getExtra().concavity = distToSeg(m1->getPos(),m2->getPos(),tmpPnt->getPos());

			if(tmpPnt->getExtra().concavity > m_tau)
				tmpholePMs.push_back(tmpPnt);


			tmpholeFeatPnts.push_back(tmpPnt);

			approxVNum++;
		}
		for (list<ply_vertex*>::iterator vlIte=approx2.begin();vlIte!=approx2.end();++vlIte)
		{
			ply_vertex* tmpPnt = vmap2[*vlIte];
			//assert(tmpPnt != m1->getPre() && tmpPnt != m2->getPre() && tmpPnt != m1->getNext() && tmpPnt != m2->getNext());
			if(tmpPnt == m1 || tmpPnt == m2)
				continue;

			//update the concavity for this approximate point
			tmpPnt->getExtra().concavity = distToSeg(m1->getPos(),m2->getPos(),tmpPnt->getPos());

			if(tmpPnt->getExtra().concavity > m_tau)
				tmpholePMs.push_back(tmpPnt);
			tmpholeFeatPnts.push_back(tmpPnt);

			approxVNum++;
		}
		if(approxVNum < 3)
		{
			tmpholeFeatPnts.push_back(furthestV);
		}
	}
	else
	{
		//find a root cut to build a cut tree
		HoleDiagonal* rootcutTmp = getCutDiagRoot(holecuts);
		draw_decoration.holecutRoot.setEndPoints(rootcutTmp->startPnt, rootcutTmp->endPnt);

		//just to record the cut diagonals
		for(vector<c_diagonal>::const_iterator dit = holecuts.begin(); dit != holecuts.end(); ++dit)
		{
			holeCutDiagonals.push_back( c_diagonal(dit->getV1(), dit->getV2()));
		}

		vector<ply_vertex*> tmpcutPoints;
		collectCutPoints(holecuts,tmpcutPoints);

		//build concavity based on the cut diagonals
		constructHierarchyConcavity(rootcutTmp);

		findFeatAndConcavityForComplexHole(hole, tmpcutPoints, tmpholePMs, tmpholeFeatPnts, m_tau, rootcutTmp);


	}
}

void c_dude::findFeatAndConcavityForComplexHole(c_ply& hole,vector<ply_vertex*>& cutPnts,list<ply_vertex*>& tmpHolePMs,list<ply_vertex*>& tmpHoleFeatPnts,double tau, HoleDiagonal* rootHoleCut)
{
	vector<HoleDiagonal*> holeDiagVec;
	hierarchyToList(rootHoleCut, holeDiagVec);

	//collect the feature points between each two cut points
	collectFeatPntBetweenCuts(hole, cutPnts, tmpHolePMs, tmpHoleFeatPnts, tau, holeDiagVec, REDUCEPOCKETMINIMUM );

	for(vector<ply_vertex*>::iterator vit = cutPnts.begin(); vit != cutPnts.end(); ++vit){
		ply_vertex* tv = *vit;
		if(tv == rootHoleCut->startPnt || tv == rootHoleCut->endPnt)
			continue;

		if(tv->isReflex())
		{
			tv->getExtra().concavity = tv->getExtra().holeCutConcavity;//distToSeg(prePM->getPos(), nextPM->getPos(), tv->getPos());// + min(prePM->getExtra().concavity, nextPM->getExtra().concavity);

			//tv->getExtra().concavity_hole = &hole;
			if(tv->getExtra().concavity > m_tau && (!REDUCEPOCKETMINIMUM))
				tmpHolePMs.push_back(tv);
		}
		tmpHoleFeatPnts.push_back(tv);
	}

	ply_vertex* rootReflexV = NULL;
	ply_vertex* rootNonReflex = NULL;
	if(rootHoleCut->startPnt->isReflex()){
		rootReflexV = rootHoleCut->startPnt;
		rootNonReflex = rootHoleCut->endPnt;
	}
	else if(rootHoleCut->endPnt->isReflex()){
		rootReflexV = rootHoleCut->endPnt;
		rootNonReflex = rootHoleCut->startPnt;
	}

	if(rootReflexV != NULL){
		//search from two directions to get two PMs to approximate concavity
		ply_vertex* preReflex = rootNonReflex->getPre();
		ply_vertex* nextReflex = rootNonReflex->getNext();
		while(!preReflex->isReflex()){
			preReflex = preReflex->getPre();
		}
		while(!nextReflex->isReflex()){
			nextReflex = nextReflex->getNext();
		}
		float tmpDist = dist2Seg(preReflex->getPos(), nextReflex->getPos(), rootNonReflex->getPos());
		rootReflexV->getExtra().concavity = distance2Point(rootReflexV->getPos(), rootNonReflex->getPos()) + tmpDist;
		if(rootReflexV->getExtra().concavity > m_tau)
			tmpHolePMs.push_back(rootReflexV);
	}

	tmpHoleFeatPnts.push_back(rootHoleCut->startPnt);
	tmpHoleFeatPnts.push_back(rootHoleCut->endPnt);

}

void c_dude::decompose_bridge(c_BPC * bpc)
{
	c_diagonal diag(bpc->getSource1(),bpc->getSource2());
//	allBridges.push_back(diag);
    //for each BPC if not convex enough, do decomposition
    c_ply ply=create_ply_from_bridge(bpc->getSource1(),bpc->getSource2());
    if(ply.getHead()->getPos()!=bpc->getSource2()->getPos()) assert(false);

    map<ply_vertex*,ply_vertex *> vmap;
    {
        ply_vertex * v=ply.getHead();
        ply_vertex * u=bpc->getSource2();
        do{
            vmap[v]=u;
            v=v->getNext();
            u=u->getPre();
        }while(v!=ply.getHead());
    }

    c_polygon polygon;
    polygon.push_back(ply);
    c_dude* dude2 = new c_dude();
    dude2->build(polygon, m_tau,false);

    //keep K best diagonals and remove the rest
    const vector<c_diagonal>& cuts=dude2->getFinalCuts();
    for(vector<c_diagonal>::const_iterator j=cuts.begin();j!=cuts.end();j++){
        const c_diagonal& dia=*j;
        c_BPC * bpc2=new c_BPC();
        //assert(bpc2);
        ply_vertex * v1=vmap[dia.getV1()];
        ply_vertex * v2=vmap[dia.getV2()];
        //assert(v1);
        //assert(v2);

        if( bpc2->build(v2,v1) == false ){
            delete bpc2;
            continue;
        }

        //not really a bpc...
        if(bpc2->getSource1()->getNext()==bpc2->getSource2()){
            delete bpc2;
            continue;
        }

        bpc->addKid(bpc2);

    }//end j
}

//
// create simplified c_ply from the active flag
//
void c_dude::buildSimpliedPly(/*const */ c_ply& from,c_ply& to)
{
    ply_vertex * ptr=from.getHead();
    bool found_concavity=false;
    bool isHole = (from.getType()==c_ply::PIN);
    to.beginPoly();
    do{

		//marked as a special vertex
		if(ptr->getExtra().flag==m_marking_flag)
		{
			const Point2d& pt=ptr->getPos();
			ply_vertex * new_v=to.addVertex(pt[0],pt[1]);

			//build the map between from vertices and to vertices
			new_v->getExtra().other_v=ptr;
			ptr->getExtra().other_v=new_v;

			if(isHole)
			{
				ptr->getExtra().setConcavity_hole(&from);
				new_v->getExtra().setConcavity_hole(&to);
			}

			if(ptr->getExtra().concavity_bpc!=NULL)
				found_concavity=true;
		}
		
        ptr=ptr->getNext();
    }
    while(ptr!=from.getHead());

    if(to.getHead()!=NULL)
    {
        to.endPoly();
    }

//    //not if a boundary without any BPC
//	if( (from.getType()==c_ply::POUT) && (found_concavity==false))
//        to.destroy();
}

//
//add a diagonal to this alpha polygon
//
void c_dude::addDiagonal(ply_vertex * s, ply_vertex * t, c_polygon& polygon)
{
    ply_vertex * v1=(polygon[0]==m_simpilifed_P[0])?s->getExtra().other_v:s;
    ply_vertex * v2=(polygon[0]==m_simpilifed_P[0])?t->getExtra().other_v:t;
    //assert(v1 && v2);

    c_diagonal diagonal(v1,v2);

//    if(v1->getExtra().parentPly==v2->getExtra().parentPly && v1->getExtra().parentPly->getType()==c_ply::PIN)
//    {
//#if DEBUG
//    	cout<<"an abnormal diagonal"<<endl;
//#endif
//    	diagonal.inHole = true;
//    }

    //if(diagonal.getWeight()==0) return; //too insignificant...

	if (v1==NULL || v2==NULL)
	{
		return;
	}
	
    m_diagonals.push_back(diagonal);
    v1->getExtra().diagonals.push_back(diagonal);
    v2->getExtra().diagonals.push_back(diagonal);
}

//isDiagonal check if a vertex pair if a diagonal in *m_reduced_P*
bool c_dude::isDiagonal( ply_vertex* s,  ply_vertex * t, c_polygon& polygon, c_polygon& initPolygon)
{
    if(s==NULL || t==NULL) return false;

    ply_vertex* v1 = (polygon[0]==m_simpilifed_P[0])?s->extra.other_v:s;
    ply_vertex* v2 = (polygon[0]==m_simpilifed_P[0])?t->extra.other_v:t;


    if(!isHoleDiagonalValid(v1,v2)) // check if both of two end points of this diagonal is in hole and they are approximated by a single line
    	return false;


    if(is_bridge_breaking(v1,v2)==false) return false;

    const Point2d& p1=v1->getPos();
    const Point2d& p2=v2->getPos();

    //check if v1v2 inside p
    Point2d mid( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2);
    if( polygon.enclosed(mid)==false ) return false;

    if(initPolygon.enclosed(mid)==false) return false;

    return true;
}

//check if u is in v's pocket
bool c_dude::is_bridge_breaking(const ply_vertex * v, const ply_vertex * u)
{
    //get bpc of v
    const c_BPC * bpc1=v->getExtra().concavity_bpc;
    const c_BPC * bpc2=u->getExtra().concavity_bpc;

	const c_ply * hole1 = v->getExtra().getConcavity_hole();
	const c_ply * hole2 = u->getExtra().getConcavity_hole();

    if(bpc1==NULL && bpc2==NULL && hole1==NULL && hole2==NULL) return false; //no bridge to break...

    if((bpc1==bpc2) && (hole1==hole2) ) return false; //same bridge or same hole...


    //check if u is in the pocket of v
    if(bpc1!=NULL && bpc2==NULL){
        //cout<<"bpc1="<<bpc1<<endl;
        //bpc1=bpc1->getRoot();
        ply_vertex * s=bpc1->getSource1()->getNext();
        ply_vertex * t=bpc1->getSource2();
        for(ply_vertex * ptr=s;ptr!=t->getNext();ptr=ptr->getNext()){
            if(ptr==u) return false;
        }
    }

    //check if v is in the pocket of u
    if(bpc1==NULL && bpc2!=NULL){
         //bpc2=bpc2->getRoot();
         ply_vertex * s=bpc2->getSource1()->getNext();
         ply_vertex * t=bpc2->getSource2();
         for(ply_vertex * ptr=s;ptr!=t->getNext();ptr=ptr->getNext()){
             if(ptr==v) return false;
         }
     }

    return true;
}


void c_dude::collectPMs(list<c_BPC *>& bpcs)
{
    typedef list<c_BPC*>::iterator IT;

    set<ply_vertex *> pms_set;  //use set to avoid duplicates
    pms_set.insert(m_PMs.begin(),m_PMs.end());
  //  m_PMs.clear();

    for(IT i=bpcs.begin();i!=bpcs.end();i++)
    {
        c_BPC* bpc=*i;
        const list<ply_vertex *>& pms=bpc->getConcavities();
        pms_set.insert(pms.begin(),pms.end());
    }

    //make sure that there is no duplicates
    m_PMs.insert(m_PMs.end(),pms_set.begin(),pms_set.end());
}
void c_dude::collectOutBoundFeatPnts()
{
	typedef list<c_BPC*>::iterator BIT;
	poutFeatPnts.clear();
	set<ply_vertex*> outFeatPntSet;

	for(BIT i=m_bpcs.begin();i!=m_bpcs.end();i++)
	{
		c_BPC* bpc=*i;

		outFeatPntSet.insert(bpc->getSource1());
		outFeatPntSet.insert(bpc->getSource2());
		const list<ply_vertex *>& pms=bpc->getConcavities();
		for(list<ply_vertex *>::const_iterator j=pms.begin();j!=pms.end();j++){
			outFeatPntSet.insert(*j);
		}

		const list<ply_vertex *>& bes=bpc->getBridgeEnds();
		for(list<ply_vertex *>::const_iterator j=bes.begin();j!=bes.end();j++){
			outFeatPntSet.insert(*j);
		}
	}//end i

	for (list<ply_vertex*>::iterator pite = convexFeatPnts.begin();pite!=convexFeatPnts.end();++pite)
	{
		outFeatPntSet.insert(*pite);
	}

    //make sure that there is no duplicates
	poutFeatPnts.insert(poutFeatPnts.end(),outFeatPntSet.begin(),outFeatPntSet.end());
	
}
void c_dude::collectTotalFeatPnts()
{
	//totalFeatPnts.clear();
	set<ply_vertex*> totalFeatPntSet;
	for(list<ply_vertex*>::iterator pite = poutFeatPnts.begin();pite!=poutFeatPnts.end();++pite)
	{
		totalFeatPntSet.insert(*pite);
	}

	for (list<ply_vertex*>::iterator pite = holeFeatPnts.begin();pite!=holeFeatPnts.end();++pite)
	{
		totalFeatPntSet.insert(*pite);
	}

	//make sure that there is no duplicates
	totalFeatPnts.insert(totalFeatPnts.end(),totalFeatPntSet.begin(),totalFeatPntSet.end());

}
//build cutset
void c_dude::buildCutSets(list<ply_vertex*>& pms)
{
    typedef list<ply_vertex*>::iterator IT;

    //build cutset for each pm
    for(IT i=pms.begin();i!=pms.end();i++)
    {
        ply_vertex * v=*i;
        if(v->getExtra().diagonals.empty()) continue; //no diagonal

        ply_vertex_extra& extra=v->getExtra();
        ply_vertex * start_v=v->getExtra().getDihedralPre();
        ply_vertex * end_v=v->getExtra().getDihedralNext();
        const Point2d& start=start_v->getPos();
        const Point2d& end=end_v->getPos();
        c_cutset::build_cutsets(v,start,end,extra.diagonals,m_tau,extra.cutsets);

        //remember
        m_cutsets.insert(m_cutsets.end(),extra.cutsets.begin(),extra.cutsets.end());

    }//end i
}

//keep K best diagonals and remove the rest
void c_dude::keepKBest(list<ply_vertex*>& pms, vector<c_diagonal>& cuts)
{
    c_knapsack knapsack(m_cutsets);
    knapsack.build();
    knapsack.getOptimalCuts(pms, m_tau, cuts);
}

////
////keep diagonals that have end points with at least tau concavity
////note: tau is scaled by the radius of P
////
//void c_dude::keepTauConcavity(float tau)
//{
//    tau = tau * getP().front().getRadius();
//
//    //go through the diagonals and remove ones that have low concavity
//    typedef vector<c_diagonal>::iterator IT;
//    for(IT  i=m_diagonals.begin();i!=m_diagonals.end();i++){
//        c_diagonal& dia=*i;
//        float concavity1=dia.getV1()->getExtra().concavity;
//        float concavity2=dia.getV2()->getExtra().concavity;
//        if(concavity1<tau && concavity2<tau){
//            IT j=i; i--;
//            m_diagonals.erase(j);
//        }
//    }//end for i
//}

//
// create a polygon from the pocket
c_ply c_dude::create_ply_from_bridge(ply_vertex * s, ply_vertex * e)
{
	ply_vertex * ptr=e;
	c_ply ply(c_ply::POUT);
	ply.beginPoly();
	do{
		const Point2d& pos=ptr->getPos();
		ply.addVertex(pos[0],pos[1]);
		ptr=ptr->getPre();
	}
	while(ptr!=s->getPre());
	ply.endPoly();
	
	//for testing
//	allPocPlys.push_back(ply);

	return ply;
}


c_plyline c_dude::bpc2polyline( const list<ply_vertex*>& vlist )
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

void c_dude::resolveIntersectedCuts(c_polygon& tmpPolygon)
{
	for(vector<c_diagonal>::iterator dit = m_cuts.begin(); dit != m_cuts.end(); ++dit)
	{
		c_diagonal& tdig = *dit;
		vector<Point2d> intersectionPositions;
		vector<pair<ply_vertex*, ply_vertex*> > intersectionEdges;
		double s[2], e[2];
		s[0] = tdig.getV1()->getPos()[0]; s[1] = tdig.getV1()->getPos()[1];
		e[0] = tdig.getV2()->getPos()[0]; e[1] = tdig.getV2()->getPos()[1];

		for(c_polygon::iterator plyit = tmpPolygon.begin(); plyit != tmpPolygon.end(); ++plyit)
		{
			c_ply& tply = *plyit;
			if(tply.canBeIgnored)
				continue;

			ply_vertex* head = tply.getHead();
			ply_vertex* pcur = head;
			ply_vertex* pnext = pcur->getNext();
			do
			{
				double cur[2], next[2], inter[2];
				cur[0] = pcur->getPos()[0]; cur[1] = pcur->getPos()[1];
				next[0] = pnext->getPos()[0]; next[1] = pnext->getPos()[1];

				char intersectCode = SegSegInt(s, e, cur, next, inter);
				Point2d interPos(inter[0], inter[1]);
				//char intersectCode = SegSegInt(tdig.getV1()->getPos(), tdig.getV2()->getPos(), pcur->getPos(), pnext->getPos(), interPos);
				if(intersectCode != 'e' && intersectCode != '0')
				{
					if(distance2Point(tdig.getV1()->getPos(), interPos) >= SMALLZERO && distance2Point(tdig.getV2()->getPos(), interPos) >= SMALLZERO)
					{
						intersectionPositions.push_back(interPos);
						intersectionEdges.push_back(make_pair(pcur, pnext));
					}
				}
				pcur = pnext;
				pnext = pnext->getNext();
			}while(pcur != head);
		}

		//the cut doesn't intersect with any edge
		if(intersectionPositions.size() == 0)
		{
			continue;
		}
		vector<Point2d>::iterator svit = intersectionPositions.begin();
		vector<Point2d>::iterator evit = intersectionPositions.begin();
		vector<pair<ply_vertex*, ply_vertex*> >::iterator svitEdge = intersectionEdges.begin();
		vector<pair<ply_vertex*, ply_vertex*> >::iterator evitEdge = intersectionEdges.begin();
		float sfarDist = FLT_MAX;
		float efarDist = FLT_MAX;

		//get the closed intersection point for both cut's end points
		vector<Point2d>::iterator vit = intersectionPositions.begin();
		vector<pair<ply_vertex*, ply_vertex*> >::iterator edgeit = intersectionEdges.begin();
		for(; vit != intersectionPositions.end() && edgeit != intersectionEdges.end(); ++vit, ++edgeit)
		{
			Point2d& tp = *vit;
			float distS = distance2Point(tdig.getV1()->getPos(), tp);
			float distE = distance2Point(tdig.getV2()->getPos(), tp);
			if(distS < sfarDist){
				sfarDist = distS;
				svit = vit;
				svitEdge = edgeit;
			}
			if(distE < efarDist){
				efarDist = distE;
				evit = vit;
				evitEdge = edgeit;
			}
		}

		//tmpPolygon: build the center and triangulation
		Point2d& sInter = *svit;//the closest point to the start vertex s
		Point2d& eInter = *evit;//the closest point to the end vertex e
		double sc[2], ec[2];
		sc[0] = (sInter[0] + tdig.getV1()->getPos()[0]) / 2;
		sc[1] = (sInter[1] + tdig.getV1()->getPos()[1]) / 2;
		ec[0] = (eInter[0] + tdig.getV2()->getPos()[0]) / 2;
		ec[1] = (eInter[1] + tdig.getV2()->getPos()[1]) / 2;
		Point2d sInterCenter(sc[0], sc[1]);
		Point2d eInterCenter(ec[0], ec[1]);
		bool sInterClosed = tmpPolygon.enclosed(sInterCenter);
		bool eInterClosed = tmpPolygon.enclosed(eInterCenter);

		pair<ply_vertex*, ply_vertex*>& sOtherEdge = *svitEdge;
		pair<ply_vertex*, ply_vertex*>& eOtherEdge=*evitEdge;
		ply_vertex* sInterOtherV =sOtherEdge.first;
		ply_vertex* eInterOtherV = eOtherEdge.first;
		if( distance2Point(tdig.getV1()->getPos(), sInterOtherV->getPos())  > distance2Point(tdig.getV1()->getPos(), sOtherEdge.second->getPos()) )
			sInterOtherV = sOtherEdge.second;
		if( distance2Point(tdig.getV2()->getPos(), eInterOtherV->getPos())  > distance2Point(tdig.getV2()->getPos(), eOtherEdge.second->getPos()) )
			eInterOtherV = eOtherEdge.second;



		if(sInterClosed && eInterClosed){
			if(sfarDist > efarDist)
				tdig.setEndPoints(tdig.getV1(), sInterOtherV);
			else
				tdig.setEndPoints(eInterOtherV, tdig.getV2());
		}
		else if(sInterClosed){
			tdig.setEndPoints(tdig.getV1(), sInterOtherV);
		}
		else if(eInterClosed){
			tdig.setEndPoints(eInterOtherV, tdig.getV2());
		}
		else
		{
			cerr<<"error in resolving intersected cuts..."<<endl;
		}
//		if(tdig.getV1()->getNext() == tdig.getV2() || tdig.getV2()->getNext() == tdig.getV1())
//		{
//			dit = m_cuts.erase(dit);
//
//		}

//		++dit;


	}
	for(vector<c_diagonal>::iterator dit = m_cuts.begin(); dit != m_cuts.end(); )
	{
		c_diagonal& tdig = *dit;
		if(tdig.getV1()->getNext() == tdig.getV2() || tdig.getV2()->getNext() == tdig.getV1())
		{
			dit = m_cuts.erase(dit);
			continue;
		}
		vector<c_diagonal>::iterator cdit = dit;
		++cdit;
		bool isRedudant = false;
		for(; cdit != m_cuts.end(); ++cdit)
		{
			c_diagonal& otherDiag = *cdit;
			if((tdig.getV1() == otherDiag.getV2() && tdig.getV2()==otherDiag.getV1()) || (tdig.getV1() == otherDiag.getV1() && tdig.getV2()==otherDiag.getV2()))
			{
				dit = m_cuts.erase(dit);
				isRedudant = true;
				break;
			}
			if(checkCutSkinny(tdig.getV1(), tdig.getV2(), m_tau))
			{
				dit = m_cuts.erase(dit);
				isRedudant = true;
				break;
			}

		}
		if(!isRedudant)
			++dit;

	}
}

void c_dude::clear()
{
	m_bpcs.clear();

    m_reduced_P.clear();    // this is a polygon with the subset of boundaries of P
    m_simpilifed_P.clear(); // this is a polygon with only tips and concavities

    m_PMs.clear(); // all pocket minima

    m_cutsets.clear();   //all cutsets generated from bpcs in build function

    m_diagonals.clear(); //diagonals are created from CDT

    m_cuts.clear(); //final cuts

	//added by Guilin
	hole_PMs.clear();//to record the pocket minimum points of the hole
	holeFeatPnts.clear();//to record the feature points in the hole
	convexFeatPnts.clear();//to record the features points on the convex of the extra-boundary
	poutFeatPnts.clear();//to record the feature points of the whole extra-boundary,
								//that is to combine the pocket features points and convex feature points
	totalFeatPnts.clear();//all the feature points of this polygon
	combine_PMs.clear();//combine the extra-boundary and hole boundary feature points
	//holeAxisPnts.clear();

	bpcStartPnts.clear();
	bpcEndPnts.clear();
	holeCutDiagonals.clear();
}


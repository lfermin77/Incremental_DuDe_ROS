#include "dude_use.h"
#include "dude.h"
#include "dude_cut.h"
#include "polygon.h"
#include "SE2d_decomp.h"
#include <vector>

#include "draw_decoration.h"

using namespace std;

extern Draw_Decoration draw_decoration;

//tau
float TauRatio = 0.9;
float MinRatio =1;

c_dude_use::c_dude_use()
{
	todo_list.clear();
}
void c_dude_use::destory()
{
	todo_list.clear();
}
void c_dude_use::addInitialPolygon(c_polygon& cpolygon)
{
	todo_list.clear();
	markParentPly(&cpolygon);
	todo_list.push_back(&cpolygon);
}
void updateCut(c_diagonal& cut,vector<c_diagonal>& allCuts,
		 vector<c_diagonal>::iterator tmpIte, c_ply* srcPly, c_ply* dstPly)
{
	vector<c_diagonal>::iterator dIte = tmpIte;
	dIte++;
	for (;dIte!=allCuts.end();++dIte)
	{
		c_diagonal& tmpDiag = *dIte;
		if(tmpDiag.getV1()->getExtra().parentPly == srcPly)
			tmpDiag.getV1()->getExtra().parentPly = dstPly;

		if(tmpDiag.getV2()->getExtra().parentPly == srcPly)
			tmpDiag.getV2()->getExtra().parentPly = dstPly;

	}

}
void test_cuts(vector<c_diagonal>& allCuts, vector<c_diagonal>::iterator ite)
{
	for(; ite != allCuts.end(); ++ite)
	{
		c_diagonal& diag = *ite;
		ply_vertex* s = diag.getV1();
		ply_vertex* e = diag.getV2();
		assert(s->getExtra().parentPly != NULL);
		assert(s->getExtra().parentPolygon != NULL);
		assert(e->getExtra().parentPly != NULL);
		assert(e->getExtra().parentPolygon != NULL);
	}
}
void c_dude_use::decomposePolygonUsingCuts(c_polygon& cpolygon, vector<c_diagonal>& allCuts, SE2d& se, bool bExtractSkeleton)
{
	addInitialPolygon(cpolygon);
	if(allCuts.size() == 0)
	{
		cerr<<"Warning: no cut for this iteration.\n";
		return;
	}

	for (vector<c_diagonal>::iterator dit = allCuts.begin();dit!=allCuts.end();++dit)
	{

		c_diagonal& tmpDiag = *dit;
		ply_vertex* v1 = dit->getV1();//cutv1->getExtra().other_v;
		ply_vertex* v2 = dit->getV2();//cutv2->getExtra().other_v;

		if(v1->getExtra().parentPly->getType()==c_ply::POUT && 
			v2->getExtra().parentPly->getType()==c_ply::POUT)
		{
			//continue;

			//cerr<<"cut: out & out\n";
			c_polygon* initPlygon = v1->getExtra().parentPolygon;
			c_polygon* initPlygon2 = v2->getExtra().parentPolygon;

			c_polygon* psubPoly1 = new c_polygon();
			c_polygon* psubPoly2 = new c_polygon();
			pair<c_polygon*,c_polygon*> subPolygons(psubPoly1,psubPoly2);

			/**************************************************/
			//apply the cut and update he corresponding information
			bool p1hasmorecut=false;
			bool p2hasmorecut=false;

			//bool b = cutPolys(subPolygons,*dit,allCuts,dit,p1hasmorecut,p2hasmorecut);
			bool b = cutPolys_new(subPolygons,*dit,allCuts,dit,p1hasmorecut,p2hasmorecut);

			//extract skeleton
			if(bExtractSkeleton)
			{
				se_m* m = se.ply2MouthMap[initPlygon];//getSEMFromTodolist
				se_m* m1 = new se_m(psubPoly1);
				se_m* m2 = new se_m(psubPoly2);
				updateMouth(m1,m2,m,tmpDiag);
				se.ply2MouthMap.insert(make_pair(psubPoly1, m1));
				se.ply2MouthMap.insert(make_pair(psubPoly2, m2));
				se.todo_list.push_back(m1);
				se.todo_list.push_back(m2);
			}


			vector<c_polygon*>::iterator itr = todo_list.begin();
			while (itr != todo_list.end()) 
			{
				if ((*itr) == initPlygon)
				{
					todo_list.erase(itr);
					break;
				}
				++itr;
			}

			se_m* otherM = se.ply2MouthMap[initPlygon];
			for(list<se_m*>::iterator sit = se.todo_list.begin(); sit != se.todo_list.end(); ++sit)
			{
				if((*sit) == otherM)
				{
					se.todo_list.erase(sit);
					break;
				}
			}

			todo_list.push_back(psubPoly1);
			todo_list.push_back(psubPoly2);
			initPlygon->clear();
			//delete initPlygon;

			if(!b)
				break;
		}
		else if(v1->getExtra().parentPly->getType()==c_ply::POUT)		
		{
			//cerr<<"out & in\n";

			c_polygon& parentPlygon = *(v1->getExtra().parentPolygon);
			c_polygon* parentPlygon1 = v2->getExtra().parentPolygon;
			c_polygon* parentPlygon2= v1->getExtra().parentPolygon;

			c_ply* ply1 = v1->getExtra().parentPly;
			c_ply* ply2 = v2->getExtra().parentPly;

			bool p1hasmorecut=false;
			bool p2hasmorecut=false;

			assert(parentPlygon1 == parentPlygon2);

			se_m* m = se.ply2MouthMap[parentPlygon2];	//getSEMFromTodolist(*initPlygon);

			merge2Holes(*dit,allCuts,dit,ply1,p1hasmorecut,p2hasmorecut);

			/*********************************************/
			//remove the redundant ply
			for (c_polygon::iterator pit = parentPlygon.begin();pit!=parentPlygon.end();
				++pit)
			{
				if(ply2==(&(*pit)))
				{
					parentPlygon.erase(pit);
					break;
				}
			}

			//updateCut(*dit,allCuts, dit, ply2, ply1);
			//markParentPly(parentPlygon1);

			if(bExtractSkeleton)
			{
				se.addHoleDiagBranch(v1, v2, m);
			}


		}
		else if(v2->getExtra().parentPly->getType()==c_ply::POUT)
		{
			//cerr<<"in & out\n";

			c_polygon& parentPlygon = *(v1->getExtra().parentPolygon);
			c_polygon* parentPlygon1 = v2->getExtra().parentPolygon;
			c_polygon* parentPlygon2= v1->getExtra().parentPolygon;

			//test_outbd_num(parentPlygon);

			//se_m* m = se.ply2MouthMap[parentPlygon1];	//getSEMFromTodolist(*initPlygon);
			se_m* m = NULL;
			if(bExtractSkeleton)
				m = se.ply2MouthMap[v1->getExtra().parentPolygon];

			c_ply* ply1 = v1->getExtra().parentPly;
			c_ply* ply2 = v2->getExtra().parentPly;

			if(parentPlygon1!=parentPlygon2)
				cerr<<"warning! the cut's two end points belong two different polygons"<<endl;
			bool p1hasmorecut=false;
			bool p2hasmorecut=false;
			merge2Holes(*dit,allCuts,dit,ply2,p1hasmorecut,p2hasmorecut);

			///////////////////////////////////////////////////////

			for (c_polygon::iterator pit = parentPlygon.begin();pit!=parentPlygon.end();
				++pit)
			{
				if(ply1==(&(*pit))){
					parentPlygon.erase(pit);
					break;
				}
			}

			//updateCut(*dit,allCuts, dit, ply1, ply2);
			//markParentPly(parentPlygon1);

			if(bExtractSkeleton)
			{
				se.addHoleDiagBranch(v1, v2, m);
			}
		}
		else if(v1->getExtra().parentPly == v2->getExtra().parentPly)
		{
			//cerr<<"in & in : same hole\n";

			//cerr<<"two end ponints of cut in a same hole boundary\n";

			c_polygon* initPlygon = v1->getExtra().parentPolygon;
			c_polygon* initPlygon2 = v2->getExtra().parentPolygon;

			//c_polygon* psubPoly1 = new c_polygon();
			//c_polygon* psubPoly2 = new c_polygon();
			//pair<c_polygon*,c_polygon*> subPolygons(psubPoly1,psubPoly2);

			/**************************************************/
			//apply the cut and update he corresponding information
			bool p1hasmorecut=false;
			bool p2hasmorecut=false;

			//test_outbd_num(*initPlygon);

			c_polygon* newPolygon = cutHole(*dit,allCuts, dit);

			todo_list.push_back(newPolygon);

			markParentPly(initPlygon);
			markParentPly(newPolygon);

			//extract skeleton
			if(bExtractSkeleton)
			{
				se_m* m = se.ply2MouthMap[initPlygon];//getSEMFromTodolist
				for(list<se_m*>::iterator sit = se.todo_list.begin(); sit != se.todo_list.end(); ++sit)
				{
					if((*sit) == m)
					{
						se.todo_list.erase(sit);
						break;
					}
				}

				se_m* m1 = new se_m(initPlygon);
				se_m* m2 = new se_m(newPolygon);
				updateMouth(m1,m2,m,tmpDiag);
				se.ply2MouthMap.insert(make_pair(initPlygon, m1));
				se.ply2MouthMap.insert(make_pair(newPolygon, m2));
				se.todo_list.push_back(m1);
				se.todo_list.push_back(m2);

				//se_m* otherM = se.ply2MouthMap[initPlygon];
			}

			//break;
			//assert(false);
		}
		//else if(v)
		else
		{
			//cerr<<"in & in : dif hole\n";

			assert(v1->getExtra().parentPly != v2->getExtra().parentPly);
			//cerr<<"warning! the cut might be not correct! (both from same ply)"<<endl;
			c_polygon& parentPlygon = *(v1->getExtra().parentPolygon);
			c_ply* ply2 = v2->getExtra().parentPly;
			c_ply* ply1 = v1->getExtra().parentPly;

			bool p1hasmorecut=false;
			bool p2hasmorecut=false;
			merge2Holes(*dit,allCuts,dit,ply1, p1hasmorecut,p2hasmorecut);
			
			se_m* m = NULL;
			if(bExtractSkeleton)
				m = se.ply2MouthMap[v1->getExtra().parentPolygon];


			//markParentPly(v1->getExtra().parentPolygon);

			for (c_polygon::iterator pit = parentPlygon.begin();pit!=parentPlygon.end();
				++pit)
			{
				if(ply2==(&(*pit))){
					parentPlygon.erase(pit);
					break;
				}
			}

			updateCut(*dit,allCuts, dit,v2->getExtra().parentPly, v1->getExtra().parentPly);
			markParentPly(&parentPlygon);

			if(bExtractSkeleton)
			{
				se.addHoleDiagBranch(v1, v2, m);
			}
		}

	}


	for(vector<c_polygon*>::iterator pit = todo_list.begin(); pit != todo_list.end(); )
	{
		c_polygon* polygon = *pit;
		if(polygon->size() == 0)
		{
			pit = todo_list.erase(pit);
			continue;

		}

		reInitializePolygon(*polygon);

		markParentPly(polygon);

		++pit;

	}

}

void updateMINMAXConcavity(c_dude& dude2d)
{
    const list<ply_vertex *>& holefp=dude2d.m_PMs;//dude.holeFeatPnts;
    for(list<ply_vertex *>::const_iterator j=holefp.begin();j!=holefp.end();j++){
    	float c=(*j)->getExtra().concavity;
    	draw_decoration.allFeatureConcavities.push_back(c);
    	if( (*j)->getExtra().isPM()==false || (*j)->getExtra().diagonals.empty() ) continue;
        cerr<<c<<endl;
        if(c<draw_decoration.MINCONCAVITY){
        	draw_decoration.MINCONCAVITY=c;
        }
        if(c>draw_decoration.MAXCONCAVITY){
        	draw_decoration.MAXCONCAVITY=c;
        }

    }
    const list<ply_vertex *>& holefp2=dude2d.hole_PMs;//dude.holeFeatPnts;
    for(list<ply_vertex *>::const_iterator j=holefp2.begin();j!=holefp2.end();j++){
        float c=(*j)->getExtra().concavity;
        draw_decoration.allFeatureConcavities.push_back(c);
    	if( (*j)->getExtra().isPM()==false || (*j)->getExtra().diagonals.empty() ) continue;
        cerr<<c<<endl;
        if(c<draw_decoration.MINCONCAVITY){
        	draw_decoration.MINCONCAVITY=c;
        }
        if(c>draw_decoration.MAXCONCAVITY){
        	draw_decoration.MAXCONCAVITY=c;
        }
    }
}

void extractSkeletonFromDiagonals(c_polygon& initPolygon, vector<c_diagonal>& initCuts,vector<c_polygon*>& finalPolygonPieces, SE2d& se, bool bExtractSkeleton)
{
	/*************************************************/
	//initialize for extracting skeletons
	if(bExtractSkeleton)
	{
		se_m* initM = new se_m(&initPolygon);
		se.initPolygon = &initPolygon;
		se.todo_list.push_back(initM);
		se.todo_listPolygon.push_back(&initPolygon);
		se.ply2MouthMap.insert(make_pair(&initPolygon, initM));
	}
	c_dude_use initUse;
	initUse.decomposePolygonUsingCuts(initPolygon, initCuts, se, bExtractSkeleton);
	finalPolygonPieces.insert(finalPolygonPieces.end(), initUse.todo_list.begin(), initUse.todo_list.end());

	if(bExtractSkeleton)
		se.UpdateSkeleton();

}

void iterativeDecompose(c_polygon& initPolygon, vector<c_diagonal>& initCuts,double tau, vector<c_polygon*>& finalPolygonPieces,
		vector<c_diagonal>& finalAllCuts, SE2d& se, bool bExtractSkeleton)
{
	float tempTau = tau;
	/*************************************************/
	//initialize for extracting skeletons
	if(bExtractSkeleton)
	{
		se_m* initM = new se_m(&initPolygon);
		se.initPolygon = &initPolygon;
		se.todo_list.push_back(initM);
		se.todo_listPolygon.push_back(&initPolygon);
		se.ply2MouthMap.insert(make_pair(&initPolygon, initM));
	}


	/*************************************************/
	finalAllCuts.insert(finalAllCuts.end(), initCuts.begin(), initCuts.end());

	c_dude_use initUse;
	initUse.decomposePolygonUsingCuts(initPolygon, initCuts, se, bExtractSkeleton);

	vector<c_polygon*> pTodolist;
	pTodolist.insert(pTodolist.end(), initUse.todo_list.begin(), initUse.todo_list.end());

	while(!pTodolist.empty()){

		tempTau = TauRatio* tempTau;
		if(tempTau < tau*MinRatio)
			tempTau = tau * MinRatio;

		vector<c_polygon*> olist;
		olist.swap(pTodolist);

		for(vector<c_polygon*>::iterator pvit = olist.begin(); pvit != olist.end(); ++pvit)
		{
			c_polygon* tmpPolygon = *pvit;

			c_dude tmpDude;
			tmpDude.build(*tmpPolygon, tempTau, true);
			draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), tmpDude.m_PMs.begin(), tmpDude.m_PMs.end());
			draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), tmpDude.hole_PMs.begin(), tmpDude.hole_PMs.end());
			updateMINMAXConcavity(tmpDude);


			//record the cuts
			if(tmpDude.m_cuts.size() > 0)
			{
				finalAllCuts.insert(finalAllCuts.end(), tmpDude.m_cuts.begin(), tmpDude.m_cuts.end());

				//record  the cutted polygon pieces
				c_dude_use tmpUse;
				tmpUse.decomposePolygonUsingCuts(*tmpPolygon, tmpDude.m_cuts, se, bExtractSkeleton);

				pTodolist.insert(pTodolist.end(), tmpUse.todo_list.begin(), tmpUse.todo_list.end());

				//delete tmpPolygon;
			}
			else
			{
				reInitializePolygon(*tmpPolygon);

				finalPolygonPieces.push_back(tmpPolygon);
			}
		}

	}

	if(bExtractSkeleton)
		se.UpdateSkeleton();
}

void decomposeMoreTimes(c_polygon& initPolygon, vector<c_diagonal>& initCuts,double tau, vector<c_polygon*>& finalPolygonPieces,
		vector<c_diagonal>& finalAllCuts, SE2d& se, bool bExtractSkeleton, int iterateTimes)
{
	/*************************************************/
	//initialize for extracting skeletons
	if(bExtractSkeleton)
	{
		se_m* initM = new se_m(&initPolygon);
		se.initPolygon = &initPolygon;
		se.todo_list.push_back(initM);
		se.todo_listPolygon.push_back(&initPolygon);
		se.ply2MouthMap.insert(make_pair(&initPolygon, initM));
	}


	/*************************************************/
	finalAllCuts.insert(finalAllCuts.end(), initCuts.begin(), initCuts.end());

	c_dude_use initUse;
	initUse.decomposePolygonUsingCuts(initPolygon, initCuts, se, bExtractSkeleton);
	finalPolygonPieces.insert(finalPolygonPieces.end(), initUse.todo_list.begin(), initUse.todo_list.end());

	for(int i = 0; i < iterateTimes; i++)
	{
		vector<c_polygon*> tmpTodolist;
		tmpTodolist.insert(tmpTodolist.end(), finalPolygonPieces.begin(), finalPolygonPieces.end());
		finalPolygonPieces.clear();

		int k = 0;
		for(vector<c_polygon*>::iterator pvit = tmpTodolist.begin(); pvit != tmpTodolist.end(); ++pvit, ++k)
		{
			c_polygon* tmpPolygon = *pvit;

			c_dude tmpDude;
			tmpDude.build(*tmpPolygon, tau,true);
			draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), tmpDude.m_PMs.begin(), tmpDude.m_PMs.end());
			draw_decoration.allFeaturePMs.insert(draw_decoration.allFeaturePMs.end(), tmpDude.hole_PMs.begin(), tmpDude.hole_PMs.end());
			updateMINMAXConcavity(tmpDude);

			//record the cuts
			if(tmpDude.m_cuts.size() > 0)
			{
				finalAllCuts.insert(finalAllCuts.end(), tmpDude.m_cuts.begin(), tmpDude.m_cuts.end());

				//record  the cutted polygon pieces
				c_dude_use tmpUse;
				tmpUse.decomposePolygonUsingCuts(*tmpPolygon, tmpDude.m_cuts, se, bExtractSkeleton);
				finalPolygonPieces.insert(finalPolygonPieces.end(), tmpUse.todo_list.begin(), tmpUse.todo_list.end());

				//delete tmpPolygon;
			}
			//}
			else
			{
				reInitializePolygon(*tmpPolygon);

				finalPolygonPieces.push_back(tmpPolygon);
			}

			//delete tmpPolygon;
		}
	}

	//update the skeleton
	if(bExtractSkeleton)
		se.UpdateSkeleton();

}

void reInitializePolygon(c_polygon& polygon)
{
	for(c_polygon::iterator plyit = polygon.begin(); plyit != polygon.end();)
		{
			c_ply& tmpPly = *plyit;

			tmpPly.build_all();
			tmpPly.doInit();

			ply_vertex* head = tmpPly.getHead();
			ply_vertex* pcur = head;
			do{
				pcur->getExtra().reInit();
				pcur = pcur->getNext();

			}while(pcur!=head);

			if(tmpPly.getSize() < 3)
			{
				plyit = polygon.erase(plyit);
				continue;
			}

			++plyit;
		}
		polygon.build_all();

}





















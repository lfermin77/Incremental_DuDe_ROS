#include "SE2d.h"
#include "SE2d_skeleton.h"
#include "SE2d_decomp.h"
#include "polygon.h"
#include "diagonal2.h"
#include "dude.h"
#include "dude_cut.h"

///////////////////////////////////////////////////////////////////////////////
SE2d::SE2d(ExtracSkeleton * es, QualityMeasure * qm)
{
    m_ES=es;
    m_QM=qm;
	pCuts = NULL;
}
SE2d::SE2d()
{
    m_ES=NULL;
    m_QM=NULL;
	pCuts = NULL;
}
SE2d::~SE2d()
{

}

///////////////////////////////////////////////////////////////////////////////
//create polys functions
void SE2d::begin()
{
    destroy();
}

void SE2d::add(c_polygon& polygon)
{
	initPolygon = new c_polygon();
	initPolygon->copy(polygon);
	se_m* initM = new se_m(initPolygon);
	todo_list.push_back(initM);

	markParentPly(initPolygon);
	todo_listPolygon.push_back(initPolygon);
	ply2MouthMap.insert(make_pair(initPolygon,initM));

//
//    todo_list.push_back(new se_m(polygon));
//	initppolygon = &polygon;
//	candidatePolygon.copy(polygon);
//	markParentPly(&candidatePolygon);
//	todo_listPolygon.push_back(&candidatePolygon);

}
void SE2d::setCuts(vector<c_diagonal>* ppcuts)
{
	pCuts = ppcuts;
	tmpIte = pCuts->begin();
}

void SE2d::end()
{
    UpdateSkeleton();
}
se_m* SE2d::getSEMFromTodolist(c_polygon& plygon)
{
//	list<se_m*>::iterator sit = todo_list.begin();
//	for (;sit!=todo_list.end();++sit)
//	{
//		if(plygon==((*sit)->cd))
//			return *sit;
//	}
	return NULL;
}
void SE2d::removeItem( se_m* tm)
{
	todo_list.remove(tm);
}
void SE2d::addHoleDiagBranch(ply_vertex* v1,ply_vertex* v2,se_m* m)
{
	c_diagonal cdiag1(v1, v1->getNext());
	Vector2d vec = (cdiag1.getV2()->getPos() - cdiag1.getV1()->getPos()).normalize();
	Vector2d n(-vec[1], vec[0]);
	const Point2d p1 = cdiag1.getV1()->getPos();
	const Point2d p2 = cdiag1.getV2()->getPos();
	se_mouth * newm = new se_mouth();
	newm->com.set((p1[0] + p2[0])/2, (p1[1] + p2[1])/2);
	newm->connect[0] = m; newm->connect[1] = NULL;//NULL;
	newm->n = n;
	newm->mouth.push_back(p1);
	newm->mouth.push_back(p2);
	m->mouth.push_back(newm);
}

///////////////////////////////////////////////////////////////////////////////
void SE2d::Build(double d)
{
    typedef list<se_m*>::iterator MIT;
    if( m_ES==NULL ){
        cerr<<"! ERROR: SE2d::Build: No Skeleton Extration Object"<<endl;
        exit(1);
    }
   // if( m_QM==NULL ) {
   //    cerr<<"! ERROR: SE2d::Build: No Quality Measure Object"<<endl;
   ///    exit(1);
   // }
    ///////////////////////////////////////////////////////////////////////////
    {//eaxtract mouth from "new" model

		c_dude dude;
		dude.build(*initPolygon,d,true);
		vector<c_diagonal>& tmpCuts = dude.m_cuts;
		for (vector<c_diagonal>::iterator dit = tmpCuts.begin();dit!=tmpCuts.end();++dit)
		{
			c_diagonal& tmpDiag = *dit;
			ply_vertex* v1 = dit->getV1();
			ply_vertex* v2 = dit->getV2();
			if(v1->getExtra().parentPly->getType()==c_ply::POUT && 
				v2->getExtra().parentPly->getType()==c_ply::POUT)
			{
				c_polygon* tmpInitPlygon = v1->getExtra().parentPolygon;
				c_polygon* tmpInitPlygon2 = v2->getExtra().parentPolygon;
				c_polygon* psubPoly1 = new c_polygon();
				c_polygon* psubPoly2 = new c_polygon();

				bool p1hasmorecut = false;
				bool p2hasmorecut = false;
				//se_m* m = getSEMFromTodolist(*initppolygon);
				se_m* m = ply2MouthMap[tmpInitPlygon];	//getSEMFromTodolist(*initPlygon);

				pair<c_polygon*,c_polygon*> subPolygons(psubPoly1,psubPoly2);
				//decomposePolygon(subPolygons,*dit,dude.m_cuts,dit,m,p1hasmorecut,p2hasmorecut);
				cutPolys(subPolygons,*dit,dude.m_cuts,dit,p1hasmorecut,p2hasmorecut);
			
				se_m* m1 = new se_m(psubPoly1);
				se_m* m2 = new se_m(psubPoly2);
				updateMouth(m1,m2,m,tmpDiag);
				ply2MouthMap.insert(make_pair(psubPoly1, m1));
				ply2MouthMap.insert(make_pair(psubPoly2, m2));

				//delete m;
				//m = NULL;
				//removeItem(m);
				todo_list.push_back(m1);
				todo_list.push_back(m2);

				//if(p1hasmorecut==false)
				//	done_list.push_back(m1);
				//if(p2hasmorecut==false)
				//	done_list.push_back(m2);

				vector<c_polygon*>::iterator itr = todo_listPolygon.begin();
				while (itr != todo_listPolygon.end())
				{
					if ((*itr) == tmpInitPlygon)
					{
						todo_listPolygon.erase(itr);
						break;
					}
					++itr;
				}
				todo_listPolygon.push_back(psubPoly1);
				todo_listPolygon.push_back(psubPoly2);
				tmpInitPlygon->clear();
			}
			else
			{
				/**************** the*********************/
				//need to add more code to deal with this case
		
				/***************************************/
				c_ply* ply1 = NULL;//v1->getExtra().parentPly
				c_ply* ply2 = NULL;
				if(v1->getExtra().parentPly->getType()==c_ply::PIN)
				{
					ply1 = v1->getExtra().parentPly;
				}
				if(v2->getExtra().parentPly->getType()==c_ply::PIN)
				{
					ply2 = v2->getExtra().parentPly;
				}
				if(ply1==NULL&&ply2!=NULL)
					ply1 = ply2;

				c_polygon* tmpInitPlygon = v1->getExtra().parentPolygon;
				//c_polygon* tmpInitPlygon2 = v2->getExtra().parentPolygon;

				bool p1hasmorecut = false;
				bool p2hasmorecut = false;
				//se_m* m = getSEMFromTodolist(*initppolygon);
				se_m* m = ply2MouthMap[tmpInitPlygon];	//getSEMFromTodolist(*initPlygon);

				merge2Holes(*dit, dude.m_cuts, dit, ply2, p1hasmorecut, p2hasmorecut);

				for (c_polygon::iterator pit = tmpInitPlygon->begin();pit!=tmpInitPlygon->end();
					++pit)
				{
					if(ply1==(&(*pit))){
						tmpInitPlygon->erase(pit);
						break;
					}
				}
				markParentPly(tmpInitPlygon);


				//se_m* m1 = new se_m(tmpInitPlygon);
				c_diagonal cdiag1(v1, v1->getNext());
				Vector2d vec = (cdiag1.getV2()->getPos() - cdiag1.getV1()->getPos()).normalize();
				Vector2d n(-vec[1], vec[0]);
				const Point2d p1 = cdiag1.getV1()->getPos();
				const Point2d p2 = cdiag1.getV2()->getPos();
				se_mouth * newm = new se_mouth();
				newm->com.set((p1[0] + p2[0])/2, (p1[1] + p2[1])/2);
				newm->connect[0] = m; newm->connect[1] = NULL;//NULL;
				newm->n = n;
				newm->mouth.push_back(p1);
				newm->mouth.push_back(p2);
				m->mouth.push_back(newm);

//				se_mouth * newm1 = new se_mouth();
//				newm1->com.set((p1[0] + p2[0])/2, (p1[1] + p2[1])/2);
//				newm1->connect[0] = NULL; newm->connect[1] = m;//NULL;
//				newm1->n = -n;
//				newm1->mouth.push_back(p1);
//				newm1->mouth.push_back(p2);
//				m->mouth.push_back(newm1);

				//c_diagonal cdiag1(v1,v1->getNext());
				//c_diagonal cdiag2(v2,v2->getNext());

				//updateSingleMouth(m1,m,cdiag1);
				//updateSingleMouth(m1,m,cdiag2);
				//ply2MouthMap.insert(make_pair(tmpInitPlygon, m1));

				//todo_list.push_back(m1);




//				vector<c_polygon*>::iterator itr = todo_listPolygon.begin();
//				while (itr != todo_listPolygon.end())
//				{
//					if ((*itr) == tmpInitPlygon)
//					{
//						todo_listPolygon.erase(itr);
//						break;
//					}
//					++itr;
//				}
//				todo_listPolygon.push_back(psubPoly1);
//				todo_listPolygon.push_back(psubPoly2);
//				tmpInitPlygon->clear();



			}
		}


    }
    ///////////////////////////////////////////////////////////////////////////
    UpdateSkeleton();
}

///////////////////////////////////////////////////////////////////////////////
void SE2d::Build(se_m* m, double d)
{
	//if(m_QM==NULL) return;
 //   //Check the quality of this skeleton
 //   double error=m_QM->quality(m);
 //   //if not good, Decompose it, and put them into the todo list.
 //   //otherwise put into done list.
 //   if( error>d ){
 //       pair<se_m*,se_m*> de=decompose(m);
 //       if( de.first==NULL ) 
 //           done_list.push_back(m);
 //       else{
 //           delete m;
 //           todo_list.push_back(de.first);
 //           todo_list.push_back(de.second);
 //       }
 //   }
 //   else{ //good enough
 //       done_list.push_back(m);
 //   }
}

void SE2d::UpdateSkeleton()
{
	if(m_ES==NULL) return;
    typedef list<se_m*>::iterator MIT;
    ///////////////////////////////////////////////////////////////////////////
    {//re-extract bone since new mouth has been added
        for( MIT im=todo_list.begin();im!=todo_list.end();im++ ){
            se_m* m=*im;
            m_ES->extract(m);
        }
    }//end re-extract
}

void SE2d::destroy()
{
    typedef list<se_m*>::iterator MIT;
    {
        for(MIT im=todo_list.begin();im!=todo_list.end();im++)
            delete *im;
        todo_list.clear();
    }
    {
        for(MIT im=done_list.begin();im!=done_list.end();im++)
            delete *im;
        done_list.clear();
    }
}

#include "dude_cut.h"
#include "polygon.h"
#include "intersection.h"
#include <map>

#define debug 0


bool checkPlyContainPnt(c_ply& targetPly,ply_vertex* targetPnt)
{
	ply_vertex* cursor = targetPly.getHead();
	do 
	{
		if(cursor==targetPnt)
			return true;
		cursor = cursor->getNext();
	} while (cursor!=targetPly.getHead());
	return false;
}
bool checkPolygonContainPnt(c_polygon& targetPolygon,ply_vertex* targetPnt)
{
	for (c_polygon::iterator pit = targetPolygon.begin();
		pit!=targetPolygon.end();++pit)
	{
		bool flag = checkPlyContainPnt(*pit,targetPnt);
		if(flag)
			return flag;
	}
	return false;
}
c_ply& getOutBound(c_polygon& polygon)
{
	for(c_polygon::iterator pit = polygon.begin(); pit != polygon.end(); ++pit)
	{
		if(pit->getType() == c_ply::POUT)
			return (*pit);
	}
	return polygon.front();
}

void addDiagnal( ply_vertex* v1, ply_vertex * v2 )
{
	ply_vertex * v1n=v1->getNext();
	ply_vertex * v2n=v2->getNext();

	ply_vertex * n11=new ply_vertex(v1->getPos());
	//ply_vertex * n12=new ply_vertex(v1->getPos());
	ply_vertex * n21=new ply_vertex(v2->getPos());
	//ply_vertex * n22=new ply_vertex(v2->getPos());

	//v1->setNext(n11);
	//n11->setNext(n22);
	//v1->setNext(n22);
	//n22->setNext(v2n);
	v1->setNext(n21);
	n21->setNext(v2n);

	n21->setPre(v1);
	v2n->setPre(n21);


	v2->setNext(n11);
	n11->setNext(v1n);

	n11->setPre(v2);
	v1n->setPre(n11);

	//v2->setNext(n21);
	//n21->setNext(n12);
	//v2->setNext(n12);
	//n12->setNext(v1n);

	//checkDegeneracy(v1,v2);
	//updateInfo(v1);
	//updateInfo(v2);
}
void markParentPly(c_ply* p,c_polygon* polygon)
{
	ply_vertex * stPnt = p->getHead();
	stPnt->getExtra().parentPly = p;
	stPnt->getExtra().parentPolygon = polygon;
	do{
		stPnt = stPnt->getNext();
		stPnt->getExtra().parentPly = p;
		stPnt->getExtra().parentPolygon = polygon;
	} while (stPnt!=p->getHead());
}
void markParentPly(c_polygon* P)
{
	for (c_polygon::iterator plyIt = P->begin();plyIt!=P->end();++plyIt)
	{
		markParentPly(&(*plyIt),P);
	}
}
c_polygon* cutHole(c_diagonal& cut_l,vector<c_diagonal>& allCuts,
		  vector<c_diagonal>::iterator tmpIte)
{
	c_polygon* newPolygon = new c_polygon();

	//the basic information of the cut
	ply_vertex* v1 = cut_l.getV1();
	ply_vertex* v2 = cut_l.getV2();

	ply_vertex* v1n = v1->getNext();
	ply_vertex* v2n = v2->getNext();

	c_polygon* initPolygon = v1->getExtra().parentPolygon;
	c_ply* initPly1 = v1->getExtra().parentPly;
	c_ply* initPly2 = v2->getExtra().parentPly;
	c_ply* outInitPly = &(getOutBound(*initPolygon));

	assert(initPly1 == initPly2);

	//add diagonal between the end points of the cut
	addDiagnal(v1,v2);

	ply_vertex* fakeV1inP2 = v2->getNext();
	ply_vertex* fakeV2inP1 = v1->getNext();


	c_ply p1(c_ply::UNKNOWN),p2(c_ply::UNKNOWN);
	p1.set(c_ply::UNKNOWN, v1); p2.set(c_ply::UNKNOWN, v2);

	float area1 = p1.getArea(), area2 = p2.getArea();

	if(area1 > 0)
	{
		p1.set(c_ply::POUT, v1);
		initPly1->set(c_ply::PIN, v2);
		p1.doInit(); initPly1->doInit();

		newPolygon->push_back(p1);
	}
	else
	{
		p2.set(c_ply::POUT, v2);
		initPly1->set(c_ply::PIN, v1);
		p2.doInit(); initPly1->doInit();

		newPolygon->push_back(p2);

		//swap
		ply_vertex* pu = v1;
		v2 = v1;
		v1 = pu;
	}

	c_ply* newPly = &(newPolygon->back());
#if 1

	/*******************************************************/
	//update the remaining cuts' end points
	v1->getExtra().parentPly = newPly;
	v1->getExtra().parentPolygon = newPolygon;
	fakeV2inP1->getExtra().parentPly = newPly;
	fakeV2inP1->getExtra().parentPolygon = newPolygon;

	fakeV1inP2->getExtra().parentPly = v2->getExtra().parentPly;
	fakeV1inP2->getExtra().parentPolygon = v2->getExtra().parentPolygon;


	//update the rest diagonals
	vector<c_diagonal>::iterator dIte = tmpIte;
	dIte++;
	for (;dIte!=allCuts.end();++dIte)
	{
		c_diagonal& tmpDiag = *dIte;
		/*****************************************/
		//check if the new generated polygon have more cuts
		ply_vertex* s = tmpDiag.getV1();
		ply_vertex* e = tmpDiag.getV2();

		Point2d midPnt( (s->getPos()[0] + e->getPos()[0])/2, (s->getPos()[1] + e->getPos()[1])/2);

		if(newPly->enclosed(midPnt))
		{
			s->getExtra().parentPly = newPly;
			s->getExtra().parentPolygon = newPolygon;
		}


		/*****************************************/
		//update the rest cuts that have v1 or v2 as end points
		if(tmpDiag.getV1()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV2();

			if(outInitPly ->enclosed(midPnt))
			//if(checkPolygonContainPnt(*(polygons.second),otherVert))
			{
				(*dIte).setEndPoints(fakeV1inP2,otherVert);
			}
		}
		if(tmpDiag.getV2()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			//if(checkPolygonContainPnt(*(polygons.second),otherVert))
			if(outInitPly->enclosed(midPnt))
			{
				(*dIte).setEndPoints(otherVert, fakeV1inP2);
			}
		}
		if(tmpDiag.getV1()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV2();

			if(newPly->enclosed(midPnt))
			//if(checkPolygonContainPnt(*(polygons.first),otherVert))
			{
				(*dIte).setEndPoints(fakeV2inP1,otherVert);
			}
		}
		if(tmpDiag.getV2()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV1();

			//if(checkPolygonContainPnt(*(polygons.first),otherVert))
			if(newPly->enclosed(midPnt))
			{
				(*dIte).setEndPoints(otherVert, fakeV2inP1);
			}
		}
	}
#endif

	return newPolygon;
}

#if debug
//double check
void test(c_ply* p1, c_ply* p2, vector<c_diagonal>& allCuts, vector<c_diagonal>::iterator tmpIte)
{
	 vector<c_diagonal>::iterator dit = tmpIte;

	 dit++;
	for(; dit != allCuts.end(); ++dit)
	{
		c_diagonal& diag = *dit;
		ply_vertex* s = diag.getV1();
		ply_vertex* e = diag.getV2();

		Point2d midPnt((s->getPos()[0]+e->getPos()[0])/2, (s->getPos()[1] + e->getPos()[1])/2);

		if(p1->enclosed(midPnt))
		{
			assert(s->getExtra().parentPly == p1);
			assert(e->getExtra().parentPly == p1);
		}
		else
		{
			assert(s->getExtra().parentPly == p2);
			assert(e->getExtra().parentPly == p2);
		}
	}
}
void test_outbd_num(c_polygon& polygon)
{
	int n = 0;
	for(c_polygon::iterator pit = polygon.begin(); pit != polygon.end(); ++pit)
	{
		c_ply& p = *pit;
		if(p.getType() == c_ply::POUT)
			n++;
	}
	cerr<<"tmp polygon has out bound: "<<n<<endl;
}
bool checkContain(c_polygon& polygon, c_ply* ply)
{
	for(c_polygon::iterator pit = polygon.begin(); pit != polygon.end(); ++pit)
	{
		c_ply* t = &(*pit);
		if(t == ply)
			return true;
	}
	return false;
}
#endif

////////////////////////////////////////////////////////////////////////////////
//the new cut polys
bool cutPolys_new(pair<c_polygon*,c_polygon*>& polygons,c_diagonal& cut_l,vector<c_diagonal>& allCuts,
			  vector<c_diagonal>::iterator tmpIte,bool& p1hasmorecut,bool& p2hasmorecut)
{
	ply_vertex* v1 = cut_l.getV1();
	ply_vertex* v2 = cut_l.getV2();
	ply_vertex* v1n = v1->getNext();
	ply_vertex* v2n = v2->getNext();

	c_polygon* initPolygon1 = v1->getExtra().parentPolygon;
	c_polygon* initPolygon2 = v2->getExtra().parentPolygon;
	c_ply* initPly1 = v1->getExtra().parentPly;
	c_ply* initPly2 = v2->getExtra().parentPly;
	assert(initPolygon1 == initPolygon2);
	assert(initPly1 == initPly2);

	addDiagnal(v1,v2);
	ply_vertex* fakeV1inP2 = v2->getNext();
	ply_vertex* fakeV2inP1 = v1->getNext();

	c_ply p1(c_ply::POUT),p2(c_ply::POUT);
	p1.set(c_ply::POUT,v1);p2.set(c_ply::POUT,v2);
	bool pb1 = p1.doInit();
	bool pb2 = p2.doInit();
	p1.re_triangulate(); p2.re_triangulate();

	polygons.first->push_back(p1);
	polygons.second->push_back(p2);

	c_ply* pply1 = &(polygons.first->back());
	c_ply* pply2 = &(polygons.second->back());

	for (c_polygon::iterator plyit = initPolygon1->begin();
		plyit!=initPolygon1->end();++plyit)
	{
		c_ply* old = &(*plyit);
		if(old == initPly1)
			continue;

		assert(old->getType() == c_ply::PIN);
		const Point2d pcenter = old->getHead()->getPos();
		if(p1.enclosed(pcenter))
		{
			polygons.first->push_back(*plyit);
		}
		else if(p2.enclosed(pcenter))
		{
			polygons.second->push_back(*plyit);
		}
		else
		{
			assert(false);
			cerr<<"split the out boundary error!"<<endl;
		}
	}
	markParentPly(polygons.first);
	markParentPly(polygons.second);



	/**************************************************************/
	//update the remaining cuts' end points
	vector<c_diagonal>::iterator dIte = tmpIte;
	dIte++;
	for(; dIte != allCuts.end(); ++dIte)
	{
		c_diagonal& tmpDiag = *dIte;

		if(tmpDiag.getV1()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			if(v1->getExtra().parentPolygon != otherVert->getExtra().parentPolygon)
				tmpDiag.setEndPoints(fakeV1inP2,otherVert);

//			//if (v1,otherVert) intersects with (v2,v1n)
//			if(SegSegInt(v1->getPos().get(),otherVert->getPos().get(),
//				v2->getPos().get(),v1n->getPos().get()))
//			{
//				(*dIte).setEndPoints(fakeV1inP2,otherVert);
//			}
		}
		if(tmpDiag.getV2()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			if(v1->getExtra().parentPolygon != otherVert->getExtra().parentPolygon)
				tmpDiag.setEndPoints(otherVert, fakeV1inP2);

//			//if (v1,otherVert) intersects with (v2,v1n)
//			if(SegSegInt(v1->getPos().get(),otherVert->getPos().get(),
//				v2->getPos().get(),v1n->getPos().get()))
//			{
//				(*dIte).setEndPoints(otherVert, fakeV1inP2);
//			}
		}
		if(tmpDiag.getV1()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			if(v2->getExtra().parentPolygon != otherVert->getExtra().parentPolygon)
				tmpDiag.setEndPoints(fakeV2inP1,otherVert);
			//if (v2,otherVert) intersects with (v1,v2n)
//			if (SegSegInt(v2->getPos().get(),otherVert->getPos().get(),
//				v1->getPos().get(),v2n->getPos().get()))
//			{
//				(*dIte).setEndPoints(fakeV2inP1,otherVert);
//			}
		}
		if(tmpDiag.getV2()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			if(v2->getExtra().parentPolygon != otherVert->getExtra().parentPolygon)
				tmpDiag.setEndPoints(otherVert, fakeV2inP1);
			//if ((v2,otherVert) intersects with (v1,v2n)
//			if (SegSegInt(v2->getPos().get(),otherVert->getPos().get(),
//				v1->getPos().get(),v2n->getPos().get()))
//			{
//				(*dIte).setEndPoints(otherVert, fakeV2inP1);
//			}
		}

		assert(tmpDiag.getV1()->getExtra().parentPolygon == tmpDiag.getV2()->getExtra().parentPolygon);
	}


	return true;
}



////////////////////////////////////////////////////////////////////////////////

bool cutPolys(pair<c_polygon*,c_polygon*>& polygons,c_diagonal& cut_l,vector<c_diagonal>& allCuts,
			  vector<c_diagonal>::iterator tmpIte,bool& p1hasmorecut,bool& p2hasmorecut)
{
	//the basic information of the cut
	ply_vertex* v1 = cut_l.getV1();
	ply_vertex* v2 = cut_l.getV2();

	ply_vertex* v1n = v1->getNext();
	ply_vertex* v2n = v2->getNext();

	c_polygon* initPolygon = v1->getExtra().parentPolygon;
	c_ply* initPly1 = v1->getExtra().parentPly;
	c_ply* initPly2 = v2->getExtra().parentPly;


	assert(initPly1 == initPly2);
	//assert(!(v1->getNext() == v2 || v2->getNext() == v1));

	//add diagonal between the end points of the cut
	addDiagnal(v1,v2);

	ply_vertex* fakeV1inP2 = v2->getNext();
	ply_vertex* fakeV2inP1 = v1->getNext();


	c_ply p1(c_ply::POUT),p2(c_ply::POUT);
	p1.set(c_ply::POUT,v1);p2.set(c_ply::POUT,v2);
	p1.doInit(); p2.doInit();
	p1.re_triangulate(); p2.re_triangulate();

	polygons.first->push_back(p1);
	polygons.second->push_back(p2);

	c_ply* pply1 = &(polygons.first->back());
	c_ply* pply2 = &(polygons.second->back());

	/*************************************************/
	//update the c_ply in the initial polygon
	bool b = true;

	map<c_ply*, c_ply*> old2new, new2old;

	for (c_polygon::iterator plyit = initPolygon->begin();
		plyit!=initPolygon->end();++plyit)
	{
		c_ply* old = &(*plyit);
		if(old == initPly1)
			continue;

		assert(old->getType() == c_ply::PIN);
		//const Point2d& pcenter = plyit->getCenter();
		const Point2d pcenter = old->getHead()->getPos();
		if(p1.enclosed(pcenter))
		{
			polygons.first->push_back(*plyit);

			old2new.insert(make_pair(old, &(polygons.first->back())));
			new2old.insert(make_pair(&(polygons.first->back()), old));
		}
		else if(p2.enclosed(pcenter))
		{
			polygons.second->push_back(*plyit);

			old2new.insert(make_pair(old, &(polygons.second->back())));
			new2old.insert(make_pair(&(polygons.second->back()), old));
		}
		else
		{
			assert(false);
			cerr<<"split the out boundary error!"<<endl;
			b = false;
		}
	}
	cerr<<"old2new size: "<<old2new.size()<<endl;

	/*******************************************************/
	//update the remaining cuts' end points

	v1->getExtra().parentPly = pply1;
	v1->getExtra().parentPolygon = polygons.first;
	fakeV2inP1->getExtra().parentPly = pply1;
	fakeV2inP1->getExtra().parentPolygon = polygons.first;

	v2->getExtra().parentPly = pply2;
	v2->getExtra().parentPolygon = polygons.second;
	fakeV1inP2->getExtra().parentPly = pply2;
	fakeV1inP2->getExtra().parentPolygon = polygons.second;


	uint flag = ply_vertex_extra::getFlagID();

	int pin = 0;
	//update the rest diagonals
	vector<c_diagonal>::iterator dIte = tmpIte;
	dIte++;
	for (;dIte!=allCuts.end();++dIte)
	{
		pin++;

		c_diagonal& tmpDiag = *dIte;
		/*****************************************/
		//check if the new generated polygon have more cuts
		ply_vertex* s = tmpDiag.getV1();
		ply_vertex* e = tmpDiag.getV2();

		c_polygon* prePolygon_s = s->getExtra().parentPolygon;
		c_polygon* prePolygon_e = e->getExtra().parentPolygon;

		if(prePolygon_s == initPolygon || prePolygon_s == polygons.first || prePolygon_s == polygons.second)
			assert(prePolygon_e == initPolygon || prePolygon_e == polygons.first || prePolygon_e == polygons.second);

		if(prePolygon_e == initPolygon || prePolygon_e == polygons.first || prePolygon_e == polygons.second)
			assert(prePolygon_s == initPolygon || prePolygon_s == polygons.first || prePolygon_s == polygons.second);

//		bool b1 = checkContain(*prePolygon_s, s->getExtra().parentPly);
//		bool b2 = checkContain(*prePolygon_e, e->getExtra().parentPly);
//		bool b3 = checkContain(*polygons.first, s->getExtra().parentPly);
//		bool b3_2 = checkContain(*polygons.first, e->getExtra().parentPly);
//		bool b4 = checkContain(*polygons.second, s->getExtra().parentPly);
//		bool b4_2 = checkContain(*polygons.second, e->getExtra().parentPly);


		Point2d midPnt((s->getPos()[0]+e->getPos()[0])/2, (s->getPos()[1] + e->getPos()[1])/2);

		if( (!pply1->enclosed(midPnt)) && (!pply2->enclosed(midPnt)))
			continue;
		if(s->getExtra().parentPolygon != initPolygon && s->getExtra().parentPolygon != polygons.first && s->getExtra().parentPolygon != polygons.second)
		{
			assert(e->getExtra().parentPolygon != initPolygon && e->getExtra().parentPolygon != polygons.first && e->getExtra().parentPolygon != polygons.second);
			continue;
		}
		if(e->getExtra().parentPolygon != initPolygon && e->getExtra().parentPolygon != polygons.first && e->getExtra().parentPolygon != polygons.second)
		{
			assert(s->getExtra().parentPolygon != initPolygon && s->getExtra().parentPolygon != polygons.first && s->getExtra().parentPolygon != polygons.second);
			continue;
		}


		/*****************************************/
		//update the rest cuts that have v1 or v2 as end points
		if(tmpDiag.getV1()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			//if(checkPolygonContainPnt(*(polygons.second),otherVert))
			if(pply2->enclosed(midPnt))
			//if(SegSegInt(v1->getPos().get(),otherVert->getPos().get(),
			//	v2->getPos().get(),v1n->getPos().get()))
			{
				//otherVert->getExtra().parentPly = pply2;
				(*dIte).setEndPoints(fakeV1inP2,otherVert);
			}
		}
		else if(tmpDiag.getV1()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			//if(checkPolygonContainPnt(*(polygons.first),otherVert))
			if(pply1->enclosed(midPnt))
			//if (SegSegInt(v2->getPos().get(),otherVert->getPos().get(),
			//	v1->getPos().get(),v2n->getPos().get()))
			{
				//otherVert->getExtra().parentPly = pply1;
				(*dIte).setEndPoints(fakeV2inP1,otherVert);
			}
		}
		else
		{
			if(s->getExtra().parentPly == initPly1)
			{
				if(pply1->enclosed(midPnt))
					s->getExtra().parentPly = pply1;
				else
					s->getExtra().parentPly = pply2;
			}
			else if(s->getExtra().parentPly != pply1 && s->getExtra().parentPly != pply2 && new2old.find(s->getExtra().parentPly) == new2old.end())
			{
				assert(s->getExtra().parentPly->getType() == c_ply::PIN);
				assert(old2new.find(s->getExtra().parentPly) != old2new.end());
				s->getExtra().parentPly = old2new[s->getExtra().parentPly];
			}
		}

		/////////////////////////////////////////////////////////////////////////////////////////

		if(tmpDiag.getV2()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			//if(checkPolygonContainPnt(*(polygons.second),otherVert))
			if(pply2->enclosed(midPnt))
			//if(SegSegInt(v1->getPos().get(),otherVert->getPos().get(),
			//		v2->getPos().get(),v1n->getPos().get()))
			{
				//otherVert->getExtra().parentPly = pply2;
				(*dIte).setEndPoints(otherVert, fakeV1inP2);
			}
		}
		else if(tmpDiag.getV2()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			//if(checkPolygonContainPnt(*(polygons.first),otherVert))
			if(pply1->enclosed(midPnt))
			//if (SegSegInt(v2->getPos().get(),otherVert->getPos().get(),
			//	v1->getPos().get(),v2n->getPos().get()))
			{
				//otherVert->getExtra().parentPly = pply1;
				(*dIte).setEndPoints(otherVert, fakeV2inP1);
			}
		}
		else
		{
			if(e->getExtra().parentPly == initPly1)
			{
				if(pply1->enclosed(midPnt))
					e->getExtra().parentPly = pply1;
				else
					e->getExtra().parentPly = pply2;
			}
			else if(e->getExtra().parentPly != pply1 && e->getExtra().parentPly != pply2 && new2old.find(e->getExtra().parentPly) == new2old.end())
			{
				assert(e->getExtra().parentPly->getType() == c_ply::PIN);
				assert(old2new.find(e->getExtra().parentPly) != old2new.end());
				e->getExtra().parentPly = old2new[e->getExtra().parentPly];
			}
		}

		if(pply1->enclosed(midPnt))
		{
			p1hasmorecut = true;
			s->getExtra().parentPolygon = polygons.first;
			e->getExtra().parentPolygon = polygons.first;
		}
		else if(pply2->enclosed(midPnt))
		{
			p2hasmorecut = true;
			s->getExtra().parentPolygon = polygons.second;
			e->getExtra().parentPolygon = polygons.second;
		}

		assert(tmpDiag.getV1()->getExtra().parentPly != NULL);
		assert(tmpDiag.getV1()->getExtra().parentPolygon != NULL);
		assert(tmpDiag.getV2()->getExtra().parentPly != NULL);
		assert(tmpDiag.getV2()->getExtra().parentPolygon != NULL);

		if(tmpDiag.getV1()->getExtra().parentPly->getType() == c_ply::POUT && tmpDiag.getV2()->getExtra().parentPly->getType() == c_ply::POUT)
			assert(tmpDiag.getV1()->getExtra().parentPly == tmpDiag.getV2()->getExtra().parentPly);
		assert(tmpDiag.getV1()->getExtra().parentPolygon == tmpDiag.getV2()->getExtra().parentPolygon);

		s->getExtra().flag = flag;
		e->getExtra().flag = flag;


		//bool bb1 = checkContain(*tmpDiag.getV1()->getExtra().parentPolygon, tmpDiag.getV1()->getExtra().parentPly);
		//bool bb2 = checkContain(*tmpDiag.getV2()->getExtra().parentPolygon, tmpDiag.getV2()->getExtra().parentPly);

		assert(tmpDiag.getV1()->getExtra().parentPolygon == tmpDiag.getV2()->getExtra().parentPolygon);
		//assert(bb1 && bb2);
#if 0
		//update for general
		if(tmpDiag.getV1()->getExtra().parentPly == initPly1)
		{
			if(pply1->enclosed(midPnt))
				tmpDiag.getV1()->getExtra().parentPly = pply1;
			else
				tmpDiag.getV1()->getExtra().parentPly = pply2;
		}
		else if(tmpDiag.getV1()->getExtra().parentPly != pply1 && tmpDiag.getV1()->getExtra().parentPly != pply2)
		{
			if(tmpDiag.getV1()->getExtra().parentPolygon == initPolygon)
			{
				assert(old2new.find(tmpDiag.getV1()->getExtra().parentPly) != old2new.end());
				tmpDiag.getV1()->getExtra().parentPly = old2new[tmpDiag.getV1()->getExtra().parentPly];
			}
		}

		///////////////////////////////////////////////////////////////
		if(tmpDiag.getV2()->getExtra().parentPly == initPly1)
		{
			if(pply2->enclosed(midPnt))
				tmpDiag.getV2()->getExtra().parentPly = pply2;
			else
				tmpDiag.getV2()->getExtra().parentPly = pply1;
		}
		else if(tmpDiag.getV2()->getExtra().parentPly != pply1 && tmpDiag.getV2()->getExtra().parentPly != pply2)
		{
			//if(pply1->enclosed(midPnt) || pply2->enclosed(midPnt))
			if(tmpDiag.getV2()->getExtra().parentPolygon == initPolygon)
			{
				assert(old2new.find(tmpDiag.getV2()->getExtra().parentPly) != old2new.end());
				tmpDiag.getV2()->getExtra().parentPly = old2new[tmpDiag.getV2()->getExtra().parentPly];
			}
		}
#endif
	}



	return b;
}
void merge2Holes(c_diagonal& cut,vector<c_diagonal>& allCuts,
				 vector<c_diagonal>::iterator tmpIte, c_ply* result_ply, bool& p1hasmorecut,bool& p2hasmorecut)
{
	ply_vertex * v1, *v2;
	v1 = cut.getV1();
	v2 = cut.getV2();
	ply_vertex* v1n = v1->getNext();
	ply_vertex* v2n = v2->getNext();

	addDiagnal(v1,v2);

	ply_vertex* fakeV1 = v2->getNext();
	ply_vertex* fakeV2 = v1->getNext();

	c_polygon* initPolygon = v1->getExtra().parentPolygon;
	fakeV1->getExtra().parentPly = result_ply;
	fakeV2->getExtra().parentPly = result_ply;

	fakeV1->getExtra().parentPolygon = v1->getExtra().parentPolygon;
	fakeV2->getExtra().parentPolygon = v1->getExtra().parentPolygon;

	v1->getExtra().parentPly = result_ply;
	v2->getExtra().parentPly = result_ply;

	result_ply->set(result_ply->getType(), v1);
	result_ply->doInit();
	result_ply->re_triangulate();

	markParentPly(result_ply, initPolygon);

	//fakeV1->getExtra().parentPly = v1->
	//update the rest diagonals
	vector<c_diagonal>::iterator dIte = tmpIte;
	dIte++;
	for (;dIte!=allCuts.end();++dIte)
	{
		c_diagonal& tmpDiag = *dIte;

		ply_vertex* s = tmpDiag.getV1();
		ply_vertex* e = tmpDiag.getV2();

		if(s->getExtra().parentPolygon != initPolygon)
		{
			assert(e->getExtra().parentPolygon != initPolygon);
			continue;
		}

		if(tmpDiag.getV1()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			//if (v1,otherVert) intersects with (v2,v1n) 
			if(SegSegInt(v1->getPos().get(),otherVert->getPos().get(),
				v2->getPos().get(),v1n->getPos().get()))
			{
				(*dIte).setEndPoints(fakeV1,otherVert);
			}
		}
		if(tmpDiag.getV2()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			//if (v1,otherVert) intersects with (v2,v1n)
			if(SegSegInt(v1->getPos().get(),otherVert->getPos().get(),
				v2->getPos().get(),v1n->getPos().get()))
			{
				(*dIte).setEndPoints(otherVert, fakeV1);
			}
		}
		if(tmpDiag.getV1()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			//if (v2,otherVert) intersects with (v1,v2n)
			if (SegSegInt(v2->getPos().get(),otherVert->getPos().get(),
				v1->getPos().get(),v2n->getPos().get()))
			{
				(*dIte).setEndPoints(fakeV2,otherVert);
			}
		}
		if(tmpDiag.getV2()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			//if ((v2,otherVert) intersects with (v1,v2n)
			if (SegSegInt(v2->getPos().get(),otherVert->getPos().get(),
				v1->getPos().get(),v2n->getPos().get()))
			{
				(*dIte).setEndPoints(otherVert, fakeV2);
			}
		}

		assert(tmpDiag.getV1()->getExtra().parentPolygon == tmpDiag.getV2()->getExtra().parentPolygon);
	}
}


///////////////////////////////////////////////////////////////////////////
#include "SE2d_decomp.h"
#include "SE2d_data.h"

///////////////////////////////////////////////////////////////////////////////
//#include "acd2d.h"
#include "dude_cut.h"

#include <cassert>

///////////////////////////////////////////////////////////////////////////////
inline double intersect
(const Point2d& p1, const Point2d& p2, const Point2d& o, const Vector2d& n)
{
	double b=n*(p2-p1);
	if( b==0 ) return -1;
	return (n*(o-p1))/b;
}

inline se_m* belongs
(se_m* m1,se_m* m2, const Point2d& p1,const Point2d& p2)
{
	///////////////////////////////////////////////////////////////////////////
	ply_vertex * head=(*(m1->cd)).front().getHead();
	ply_vertex * ptr=head;
	do{
		if( ptr->getPos().almost_equ(p1) && 
			ptr->getPre()->getPos().almost_equ(p2) ) return m1;
		if( ptr->getPos().almost_equ(p2) && 
			ptr->getPre()->getPos().almost_equ(p1) ) return m1;

		ptr=ptr->getNext();
	}while( ptr!=head );
	///////////////////////////////////////////////////////////////////////////
	head=(*(m2->cd)).front().getHead();
	ptr=head;
	do{
		if( ptr->getPos().almost_equ(p1) && 
			ptr->getPre()->getPos().almost_equ(p2) ) return m2;
		if( ptr->getPos().almost_equ(p2) && 
			ptr->getPre()->getPos().almost_equ(p1) ) return m2;

		ptr=ptr->getNext();
	}while( ptr!=head );
	///////////////////////////////////////////////////////////////////////////
	return NULL;
}

inline bool checkBelongs(se_m* tmpm, const Point2d& p1, const Point2d& p2)
{
	///////////////////////////////////////////////////////////////////////////
	ply_vertex * head=(*(tmpm->cd)).front().getHead();
	ply_vertex * ptr=head;
	do{
		if( ptr->getPos().almost_equ(p1) &&
			ptr->getPre()->getPos().almost_equ(p2) ) return true;
		if( ptr->getPos().almost_equ(p2) &&
			ptr->getPre()->getPos().almost_equ(p1) ) return true;

		ptr=ptr->getNext();
	}while( ptr!=head );

	return false;
}

//reassign mouths in m to m1 and m2
inline void splitMouth
(se_m* m1,se_m* m2,se_m* m,const c_diagonal& dia,const Vector2d& n)
{
	const Point2d& o=dia.getV1()->getPos();//.v[0];

	//for each mouth in m
	typedef list<se_mouth*>::iterator MIT;
	for( MIT im=m->mouth.begin();im!=m->mouth.end();im++ ){
		se_mouth * mt=*im;

		const Point2d& p1=mt->mouth.front();
		const Point2d& p2=mt->mouth.back();		

		bool m1contain = checkBelongs(m1, p1, p2);
		bool m2contain = checkBelongs(m2, p1, p2);
		if(m1contain && m2contain)
		{
			mt->connect[0] = m1;
			mt->connect[1] = m2;

			m1->mouth.push_back(mt);
			m2->mouth.push_back(mt);
			continue;
		}

		se_m* target = NULL;//belongs(m1,m2,p1,p2);
		if(m1contain)
			target = m1;
		if(m2contain)
			target = m2;

		//mouth mt is not intersected by the cutting line
		if( target!=NULL ){
			///////////////////////////////////////////////////////////////////
			// Assign the mouth to m1 or m2
			if( mt->connect[0]==m ) mt->connect[0]=target;
			else mt->connect[1]=target;
			target->mouth.push_back(mt);
			continue; 
		}
		///////////////////////////////////////////////////////////////////
		//mouth mt is intersected with the cutting line
		//split a mouth into two and assign the mouth to m1 or m2

		cout<<"test"<<endl;
		double u=intersect(p1,p2,o,n);

		Point2d newp;
		for(int i=0;i<2;i++) newp[i]=(1-u)*p1[i]+u*p2[i];
		double dot=(p1-o)*n;
		const Point2d& up=(dot>0)?p1:p2;
		const Point2d& down=(dot>0)?p2:p1;
		
		double u_d=(up-newp).normsqr();
		double d_d=(down-newp).normsqr();

		if( u_d>d_d ) m1->mouth.push_back(mt);
		else  m2->mouth.push_back(mt);
	}
}

inline void newMouth
(se_m* m1,se_m* m2,const c_diagonal& dia,const Vector2d& n)
{
	const Point2d p1=dia.getV1()->getPos();//v[0];
	const Point2d p2=dia.getV2()->getPos();//v[1];
	se_mouth * m=new se_mouth();
	//create mouth
	m->com.set((p1[0]+p2[0])/2,(p1[1]+p2[1])/2);
	m->connect[0]=m1; m->connect[1]=m2;
	m->n=n;
	m->mouth.push_back(p1);
	m->mouth.push_back(p2);
	//put mouth to m1 abd m2
	m1->mouth.push_back(m);
	m2->mouth.push_back(m);
}

 void updateMouth
(se_m* m1,se_m* m2,se_m* m,const c_diagonal& dia)
{
	Vector2d vec=(dia.getV2()->getPos()-dia.getV1()->getPos()).normalize();
	Vector2d n(-vec[1],vec[0]);

	//split mouth if there are mouths splited
	splitMouth(m1,m2,m,dia,n);

	//add new mouth between m1 and m2
	newMouth(m1,m2,dia,n);
}


 /**************************************************************************************
  * deal with the case that having no hole
  *************************************************************************************/

 //reassign mouths in m to m1 and m2
 inline void splitSingleMouth
 (se_m* m1,se_m* m,const c_diagonal& dia,const Vector2d& n)
 {
 	const Point2d& o=dia.getV1()->getPos();//.v[0];

 	//for each mouth in m
 	typedef list<se_mouth*>::iterator MIT;
 	for( MIT im=m->mouth.begin();im!=m->mouth.end();im++ ){
 		se_mouth * mt=*im;

 		const Point2d& p1=mt->mouth.front();
 		const Point2d& p2=mt->mouth.back();

 		se_m* target=belongs(m1,m1,p1,p2);
 		//mouth mt is not intersected by the cutting line
 		if( target!=NULL ){
 			///////////////////////////////////////////////////////////////////
 			// Assign the mouth to m1 or m2
 			if( mt->connect[0]==m ) mt->connect[0]=target;
 			else mt->connect[1]=target;
 			target->mouth.push_back(mt);
 			continue;
 		}
 		///////////////////////////////////////////////////////////////////
 		//mouth mt is intersected with the cutting line
 		//split a mouth into two and assign the mouth to m1 or m2

 		double u=intersect(p1,p2,o,n);

 		Point2d newp;
 		for(int i=0;i<2;i++) newp[i]=(1-u)*p1[i]+u*p2[i];
 		double dot=(p1-o)*n;
 		const Point2d& up=(dot>0)?p1:p2;
 		const Point2d& down=(dot>0)?p2:p1;

 		double u_d=(up-newp).normsqr();
 		double d_d=(down-newp).normsqr();

 		m1->mouth.push_back(mt);

 		//if( u_d>d_d ) m1->mouth.push_back(mt);
 		//else  m2->mouth.push_back(mt);
 	}
 }

 inline void newSingleMouth
 (se_m* m1,const c_diagonal& dia,const Vector2d& n)
 {
 	const Point2d p1=dia.getV1()->getPos();//v[0];
 	const Point2d p2=dia.getV2()->getPos();//v[1];
 	se_mouth * m=new se_mouth();
 	//create mouth
 	m->com.set((p1[0]+p2[0])/2,(p1[1]+p2[1])/2);
 	m->connect[0]=m1; m->connect[1]=m1;
 	m->n=n;
 	m->mouth.push_back(p1);
 	m->mouth.push_back(p2);
 	//put mouth to m1 abd m2
 	m1->mouth.push_back(m);
 	//m2->mouth.push_back(m);
 }


void updateSingleMouth(se_m* m1,se_m* m,const c_diagonal& dia)
{
	Vector2d vec=(dia.getV2()->getPos()-dia.getV1()->getPos()).normalize();
	Vector2d n(-vec[1],vec[0]);

	//split mouth if there are mouths splited
	splitSingleMouth(m1,m,dia,n);

	//add new mouth between m1 and m2
	newSingleMouth(m1,dia,n);
}





list<Point2d> BRIDGES;
//#include "cd2d_bridge.h"

//pair<se_m*,se_m*> decompose(se_m * m)
//{
//	///////////////////////////////////////////////////////////////////////////
//	//Split the model
//	double tau=0;
//
//    HybridMeasurement2 * measure=(HybridMeasurement2 *)
//    ConcavityMeasureFac::createMeasure("hybrid2");
//	measure->setTau(tau);
//
//	cd_2d cd(true); //true will ask cd_2d to store cuts
//	cd.beginPolys();
//	typedef cd_polygon::iterator PIT;
//	for( PIT ip=m->cd.begin();ip!=m->cd.end();ip++ )
//		cd.addPolys(*ip);
//	cd.endPolys();
//	cd.decompose(tau,measure);
//	delete measure;
//	///////////////////////////////////////////////////////////////////////////
//	//check results
//	const list<cd_polygon>& lp=cd.getTodoList();
//	if( lp.size()!=2 ) return pair<se_m*,se_m*>(NULL,NULL);
//
//	///////////////////////////////////////////////////////////////////////////
//	//create new model and assign them mouths
//	se_m* m1=new se_m(lp.front());
//	se_m* m2=new se_m(lp.back());
//
//	const list<cd_diagonal>& dia=cd.getDiagonal();
//	updateMouth(m1,m2,m,dia.front());
//
//	return pair<se_m*,se_m*>(m1,m2);
//}

void decomposePolygon(pair<c_polygon*,c_polygon*>& polygons,c_diagonal& cut_l, 
					  vector<c_diagonal>& allCuts,vector<c_diagonal>::iterator tmpIte,se_m* m,bool& p1hasmorecut,bool& p2hasmorecut)
{
	p1hasmorecut = false;
	p2hasmorecut = false;

	ply_vertex* v1 = cut_l.getV1();
	ply_vertex* v2 = cut_l.getV2();

	ply_vertex* v1n = v1->getNext();
	ply_vertex* v2n = v2->getNext();

	addDiagnal(v1,v2);

	c_ply p1(c_ply::POUT), p2(c_ply::POUT);
	p1.set(c_ply::POUT,v1);p2.set(c_ply::POUT,v2);

	ply_vertex* fakeV1inP2 = v2->getNext();
	ply_vertex* fakeV2inP1 = v1->getNext();

	//update the rest diagonals
	vector<c_diagonal>::iterator dIte = tmpIte;
	dIte++;
	for (;dIte!=allCuts.end();++dIte)
	{
		c_diagonal& tmpDiag = *dIte;
		if(tmpDiag.getV1()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			if(checkPlyContainPnt(p2,otherVert))
			{
				p2hasmorecut = true;
				(*dIte).setEndPoints(fakeV1inP2,otherVert);
			}
		}
		if(tmpDiag.getV2()==v1)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			if(checkPlyContainPnt(p2,otherVert))
			{
				p2hasmorecut = true;
				(*dIte).setEndPoints(fakeV1inP2,otherVert);
			}
		}
		if(tmpDiag.getV1()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV2();
			if(checkPlyContainPnt(p1,otherVert))
			{
				p1hasmorecut = true;
				(*dIte).setEndPoints(fakeV2inP1,otherVert);
			}
		}
		if(tmpDiag.getV2()==v2)
		{
			ply_vertex* otherVert = tmpDiag.getV1();
			if(checkPlyContainPnt(p1,otherVert))
			{
				p1hasmorecut = true;
				(*dIte).setEndPoints(fakeV2inP1,otherVert);
			}
		}
	}

	polygons.first->push_back(p1);
	polygons.second->push_back(p2);
}
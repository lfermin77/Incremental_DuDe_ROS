#ifndef _SE2d_H_
#define _SE2d_H_

#include <list>
#include <vector>
using namespace std;

#include "SE2d_data.h"
//#include "dude_use.h"

class ExtracSkeleton;
class QualityMeasure;

class SE2d {

public:

	///////////////////////////////////////////////////////////////////////////
	SE2d();
	SE2d(ExtracSkeleton * es, QualityMeasure * qm);
	~SE2d();

	///////////////////////////////////////////////////////////////////////////
	//create polys functions
	void begin();
	void add(c_polygon& polygon);
	void end();

	//////////////////////////////////////////////////////////////////////////
	//set cuts
	void setCuts(vector<c_diagonal>* ppcuts);


	///////////////////////////////////////////////////////////////////////////
	void Build(double d);

	///////////////////////////////////////////////////////////////////////////
	//Access
	const list<se_m*>& getTodoList() const { return todo_list; }
	const list<se_m*>& getDoneList() const { return done_list; }
	void setExtracSkeleton(ExtracSkeleton * es){ m_ES=es; }
	void setQualityMeasure(QualityMeasure * qm){ m_QM=qm; }

public:

	void UpdateSkeleton();
	void Build(se_m* m, double d);
	void destroy();
	se_m* getSEMFromTodolist(c_polygon& plygon);

public:

	//a list of models, todo and done
	list<se_m*> todo_list;
	list<se_m*> done_list;

	vector<c_diagonal>* pCuts;
	vector<c_diagonal>::iterator tmpIte;

	vector<c_polygon*> todo_listPolygon;
	//c_dude_use dude_use;
	
	//c_polygon* initppolygon;
	//c_polygon candidatePolygon;
	c_polygon* initPolygon;

	//the skeleton
	SKG skeleton;

	ExtracSkeleton * m_ES;
	QualityMeasure * m_QM;

	//add by Guilin
	void removeItem( se_m* tm);
	map<c_polygon*, se_m*> ply2MouthMap;

	void addHoleDiagBranch(ply_vertex* v1,ply_vertex* v2,se_m* m);
};

#endif //_SE2d_H_

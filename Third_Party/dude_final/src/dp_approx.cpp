#include "dp_approx.h"
#include "polygon.h"
#include "polyline.h"
#include "holediag.h"


double distToSeg(const Point2d& s,const Point2d& t,const Point2d& p)
{
	Vector2d v = s-t;
	Vector2d u = p-t;
	Vector2d n (v[1],-v[0]);
	float n_norm=n.norm();

	if(n_norm==0) return 0;

	return fabs(n*u/n_norm);
}


void dpReduction(ply_vertex* stPnt,ply_vertex* endPnt,double tolerance,list<ply_vertex*>& approxList)
{
	double maxDistance = -1.0;
	ply_vertex* pVMax = NULL;
	ply_vertex* pcursor = stPnt->getNext();
	while(pcursor != endPnt){
		double tmpDist = distToSeg(stPnt->getPos(),endPnt->getPos(),pcursor->getPos());
		if(tmpDist > maxDistance)
		{
			maxDistance = tmpDist;
			pVMax = pcursor;
		}
		pcursor = pcursor->getNext();
	}

	if(maxDistance <= tolerance)
		return;
	else{
		dpReduction(stPnt,pVMax,tolerance,approxList);
		approxList.push_back(pVMax);
		dpReduction(pVMax,endPnt,tolerance,approxList);
	}
}
void dp_approximate(const c_plyline& plyline, double tau, list<ply_vertex*>& approx)
{
	approx.clear();
	ply_vertex* stPnt = plyline.getHead();
	ply_vertex* endPnt = plyline.getTail();
	dpReduction(stPnt,endPnt,tau,approx);
	approx.push_front(stPnt);
	approx.push_back(endPnt);
}
//startV would be able to get endV by using getNext(), the approx doesn't include startV and endV
void dp_approximate(ply_vertex* startV, ply_vertex* endV, double tau, list<ply_vertex*>& approx)
{
	approx.clear();
//	ply_vertex* stPnt = plyline.getHead();
//	ply_vertex* endPnt = plyline.getTail();
	dpReduction(startV,endV,tau,approx);
}
void approximateMedthod(const c_plyline& plyline, double tau, list<ply_vertex*>& approx, bool bUseDpApprox )
{
	if(!bUseDpApprox)
		approximate(plyline,tau,approx);
	else
		dp_approximate(plyline,tau,approx);
}


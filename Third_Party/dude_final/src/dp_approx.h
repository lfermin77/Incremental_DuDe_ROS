#ifndef _DP_APPROX_H_
#define _DP_APPROX_H_

#include "polygon.h"
#include "polyline.h"
#include "poly_approx.h"
#include <list>

using namespace std;


void approximateMedthod(const c_plyline& plyline, double tau, list<ply_vertex*>& approx, bool bUseDpApprox = false);



double distToSeg(const Point2d& s,const Point2d& t,const Point2d& p);


//using the douglas peucker compression algorithm
void dp_approximate(const c_plyline& plyline, double tau, list<ply_vertex*>& approx);


//startV would be able to get endV by using getNext(), the approx doesn't include startV and endV
void dp_approximate(ply_vertex* startV, ply_vertex* endV, double tau, list<ply_vertex*>& approx);


#endif

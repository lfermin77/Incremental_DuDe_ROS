#ifndef _SE2D_UTIL_H_
#define _SE2D_UTIL_H_

#include <Point.h>
using namespace mathtool;

//check if line segmenet q1q2 intersects segment p1p2
inline bool intersect
(const Point2d& q1, const Point2d& q2, const Point2d& p1, const Point2d& p2)
{

	Vector2d qv=(q2-q1);//.normalize();
	Vector2d qn(-qv[1],qv[0]);
	double ps1=(p1-q1)*qn;
	double ps2=(p2-q1)*qn;
	if( ps1*ps2>=-1e-10 ) return false;

	Vector2d pv=(p2-p1);//.normalize();
	Vector2d pn(-pv[1],pv[0]);
	double qs1=(q1-p1)*pn;
	double qs2=(q2-p1)*pn;
	if( qs1*qs2>=-1e-10 ) return false;

	return true;
}

//compute the distace from q to line segment p1p2
inline double dist(const Point2d& p1, const Point2d& p2, const Point2d& q)
{
	Vector2d p2_p1=p2-p1;
	Vector2d q_p1=q-p1;
	double t=(q_p1*p2_p1)/(p2_p1*p2_p1);
	if( t>1 ) t=1;
	else if( t<0 ) t=0;
	Point2d x((1-t)*p1[0]+t*p2[0], (1-t)*p1[0]+t*p2[0]);
	return (q-x).norm();
}

#endif //_SE2D_UTIL_H_
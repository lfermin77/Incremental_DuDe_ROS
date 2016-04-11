
#ifndef _DUDE_UTIL_H
#define _DUDE_UTIL_H

#include "polygon.h"
#include "diagonal2.h"
#include "Point.h"

// remove the diagonals and cuts from the vertice of the given polygon
inline void clearVerticesDiag(c_polygon& tpolygon)
{
	for(c_polygon::iterator pit = tpolygon.begin(); pit != tpolygon.end(); ++pit)
	{
		ply_vertex* pv = pit->getHead();
		do{
			pv->getExtra().cutsets.clear();
			pv->getExtra().diagonals.clear();
			pv = pv->getNext();
		}
		while(pv != pit->getHead());
	}//end for pit
}

//compute distance from p to segment (s,t)
inline float dist2Seg(const Point2d& s, const Point2d& t, const Point2d& p)
{
    Vector2d v = s-t;
    Vector2d u = p-t;
    Vector2d n (v[1],-v[0]);
    float n_norm=n.norm(); //is also v_norm
    if(n_norm==0) return u.norm();

    float d=(u*v/n_norm);
    if(d<0){
        return u.norm();
    }
    if(d>n_norm){
        return (p-s).norm();
    }

    return fabs(n*u/n_norm);
}
inline float distance2Point(const Point2d & a,const Point2d & b)
{
	return sqrtf((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]));
}
#endif

#pragma once

#include "polyline.h"

//
//given a convex polygon (boundary), 
//return the approximation that has min(sum(d_i)) for all d_i<tau
//where d_i is the distance from the approximation to the input convex polygon
//
void approximate(const c_plyline& plyline, double tau, list<ply_vertex*>& approx);


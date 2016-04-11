//------------------------------------------------------------------------------
//  Copyright 2007-2008 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _2D_CONVEX_HULL_H_
#define _2D_CONVEX_HULL_H_

///////////////////////////////////////////////////////////////////////////////
// This convex hull implemetation realizes the idea from 
// A. Melkman, "On-line construction of the convex hull of a simple polygon", 
// Info. Proc. Letters 25, 11-12 (1987)
///////////////////////////////////////////////////////////////////////////////

#include "polygon.h"

//
// compute the convex hull of the given (subset) polygon
//
// e mush be reachable from s
//
void hull2d(ply_vertex * s, ply_vertex * e, list<ply_vertex*>& hull );

#endif //_2D_CONVEX_HULL_H_


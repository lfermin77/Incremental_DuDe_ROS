// $Id: BasicDefns.h,v 1.2 2004/08/17 22:05:13 neilien Exp $
/////////////////////////////////////////////////////////////////////
//
//  BasicDefns.h
//
//  General Description
//      This is a set of VERY BASIC useful definitions.
//  They shouldn't be particularly specific to any application.
//
//  Created
//      8/25/98  Lucia K. Dale
//  Last Modified By:
//      8/29/98  Lucia K. Dale
//
/////////////////////////////////////////////////////////////////////

namespace graph{

#ifndef BasicDefns_h
#define BasicDefns_h

#define PI              3.14159
#define TWOPI           (PI*2.0)

#ifndef bool
// typedef int bool;  //aCC in parasol error
 #define true 1
 #define false 0
#endif

#ifndef OK
#define OK  0
#endif

#ifndef ERROR
#define ERROR -5
#endif

//---------------------------------------------------------------
// Legal types of collision detection data structures
//---------------------------------------------------------------
                        // Collision Detecters
enum cd_predefined {    //--------------------
        CSTK,           // c-space toolkit
        VCLIP,          // voronoi clip 
        CD_USER1};      // first user defined cd set, if any
};

#endif


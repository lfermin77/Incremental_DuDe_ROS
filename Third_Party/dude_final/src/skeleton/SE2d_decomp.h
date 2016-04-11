#ifndef _SE2D_DECOMP_H_
#define _SE2D_DECOMP_H_

#include <list> // for pair
#include "diagonal2.h"
#include "polygon.h"
#include "SE2d_data.h"
using namespace std;

class se_m;
//pair<se_m*,se_m*> decompose(se_m * m);

void updateMouth
(se_m* m1,se_m* m2,se_m* m,const c_diagonal& dia);

void updateSingleMouth(se_m* m1,se_m* m,const c_diagonal& dia);


void decomposePolygon(pair<c_polygon*,c_polygon*>& polygons,c_diagonal& cut_l, 
					  vector<c_diagonal>& allCuts,vector<c_diagonal>::iterator tmpIte,se_m* m,bool& p1hasmorecut,bool& p2hasmorecut);
#endif //_SE2D_DECOMP_H_
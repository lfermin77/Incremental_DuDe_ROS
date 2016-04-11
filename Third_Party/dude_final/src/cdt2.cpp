#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K>                     Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>           Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;
typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point          cdtPoint;
typedef CDT::Vertex_handle   Vertex_handle;

#include <cassert>
#include <iostream>

#include <algorithm>
#include <map>
using namespace std;

#include "cdt2.h"
#include "polygon.h"
#include "intersection.h"

bool c_cdt::build(c_dude& ap, c_polygon& sp, c_polygon& initPolygon)
{
    CDT cdt;
    map<const ply_vertex*,Vertex_handle> v2cgal;
    map<Vertex_handle, ply_vertex*> cgal2v;

    //register vertices of p
    uint vsize=sp.getSize();
    for(uint i=0;i<vsize;i++){
        const Point2d& pt=sp[i]->getPos();
        cdtPoint cdt_pt(pt[0],pt[1]);
        Vertex_handle handler=cdt.push_back(cdt_pt);
        v2cgal[ sp[i] ]=handler;
        cgal2v[handler]=sp[i];
    }

    //add constraints to CGAL CDT
    for(c_polygon::const_iterator i=sp.begin();i!=sp.end();i++){
        ply_vertex * ptr=i->getHead();
        do{
            //add polygon boundary
            ply_vertex * next=ptr->getNext();
            Vertex_handle h1=v2cgal[ptr];
            Vertex_handle h2=v2cgal[next];
            cdt.insert_constraint(h1,h2);
            ptr=next;
        }while(ptr!=i->getHead());
    }

    //add existing diagonals as additional constraints
    const vector<c_diagonal>& diagonals=ap.getDiagonals();
    for(vector<c_diagonal>::const_iterator i=diagonals.begin();i!=diagonals.end();i++)
    {
		//cout<<"the polygon already has some cuts before delaunay triangulation"<<endl;
        Vertex_handle h1=v2cgal[i->getV1()];
        Vertex_handle h2=v2cgal[i->getV2()];
        cdt.insert_constraint(h1,h2);
    }

    //get diagonals back from CDT
    if(!cdt.is_valid()){ cerr<<"! Warning: GDT failed"<<endl; return false; }

    //
    typedef CDT::Finite_edges_iterator IT;
    for(IT eit = cdt.finite_edges_begin(); eit != cdt.finite_edges_end(); ++eit)
    {
        if (cdt.is_constrained(*eit) == false ) //not a constraint
        {
            Vertex_handle v1 = eit->first->vertex(cdt.cw(eit->second));
            Vertex_handle v2 = eit->first->vertex(cdt.ccw(eit->second));

            ply_vertex* pv1=cgal2v[v1];
            ply_vertex* pv2=cgal2v[v2];

			if (pv1==NULL || pv2==NULL)
				continue;
			
			if(ap.isDiagonal(pv1, pv2, sp,sp)){
				ap.addDiagonal(pv1,pv2,sp);
			}
//			//ap.addDiagonal(pv1,pv2,sp);
//            if( ap.isDiagonal(pv1,pv2,sp) )
//            {
//                ap.addDiagonal(pv1,pv2,sp);
//            }

        }
    }

    return true;
}


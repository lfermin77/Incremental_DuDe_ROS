#ifndef _SE2d_DATA_H_
#define _SE2d_DATA_H_

///////////////////////////////////////////////////////////////////////////////
/**
 * @file This file contains several data structures that commonly used.
 */
 
#include "graph/Graph.h"
//using namespace graph;

//#include "Graph.h"

///////////////////////////////////////////////////////////////////////////////
#include <Point.h>
#include <Vector.h>
using namespace mathtool;

///////////////////////////////////////////////////////////////////////////////
// Convex Decomp data
#include "polygon.h"

using namespace graph;
/******************************************************************************
*
*
* Skeleton (defined using graph)
*
*
******************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Skeleton node (Joint)
///////////////////////////////////////////////////////////////////////////////

struct sk_node {
    sk_node(){ /*m_type=ERROR;*/ }
    bool operator==( const sk_node & other ) const { 
        return m_pos.almost_equ(other.m_pos); 
    }
    static sk_node InvalidData(){ 
        return sk_node(); 
    }

    Point2d m_pos;
	/*
    enum Type{ On_Mouth, On_MA, ERROR };
    Type m_type;
	*/
};

//operators
inline ostream& operator<<(ostream& out, const sk_node & node ){
	out<<node.m_pos; return out;
}

inline istream& operator>>(istream& in, sk_node & node ){ 
	in>>node.m_pos; return in; 
}

///////////////////////////////////////////////////////////////////////////////
// Skeleton edge (Link)
///////////////////////////////////////////////////////////////////////////////

struct sk_edge {
    sk_edge(double w=0){ weight=w; }
    double Weight() const { return weight; }
    static sk_edge InvalidWeight(){ return sk_edge(-1); }
    bool operator==( const sk_edge & other ) const { return weight==other.weight; }
    static double MaxWeight(){ return 1e20; }
    double weight;
};

//operators
inline ostream& operator<<(ostream& out, const sk_edge & edge ){ 
	out<<edge.weight<<" "; 
	return out; 
}

inline istream& operator>>(istream& in, sk_edge & edge ){ 
	in>>edge.weight; return in; 
}

///////////////////////////////////////////////////////////////////////////////
// The Skeleton 
///////////////////////////////////////////////////////////////////////////////

//weighted directed graph

typedef UG<sk_node,sk_edge, BaseGraph<sk_node,sk_edge> > ug;
typedef NMG<sk_node,sk_edge, BaseGraph<sk_node,sk_edge> > nmg;
typedef WG<sk_node,sk_edge, BaseGraph<sk_node,sk_edge> > wg;
typedef Graph<ug, nmg, wg, sk_node, sk_edge> SKG;

//typedef Graph<UG<sk_node,sk_edge>,NMG<sk_node,sk_edge>,WG<sk_node,sk_edge>,sk_node,sk_edge> SKG;
//typedef WeightedGraph<sk_node,sk_edge> SKG;
//typedef WG<sk_node, sk_edge> SKG;

/******************************************************************************
*
*
* SubModel Graph (the connection relation ship between sub models).
*
*
******************************************************************************/


/////////////////////////////////////////////////////////////////////////////////
///Mouth data structure, (i.e., graph edge..)
class se_m; //pre declare..
class se_mouth{
public:
    se_mouth(){ connect[0]=connect[1]=NULL; }

    se_m * connect[2];      ///< two models this mouth connects to.
    list<Point2d> mouth;    ///< the id for vertices which defines this mouth.
    Point2d com;            ///< center of mass of mouth
	Vector2d n;             //normal
};

/////////////////////////////////////////////////////////////////////////////////
///Submodel data structure.
class cd_m;
class se_m{
public:
    se_m(c_polygon* m){cd=m;}
	~se_m(){}
    
	c_polygon* cd;       //the model for convex decomp.

    /**
     * a list of mouth associsted with this submode.
     * each item in the "mouth" list is a link to the other submodel
     * which connected through Mouth.
     */
    list<se_mouth *> mouth;

	SKG skeleton;
};

#endif //_DATASTRUCTURE_H_
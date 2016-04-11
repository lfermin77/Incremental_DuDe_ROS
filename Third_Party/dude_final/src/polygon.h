//------------------------------------------------------------------------------
//  Copyright 2010-2015 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#pragma once
#ifndef _MKSUM2D_POLYGON_H_
#define _MKSUM2D_POLYGON_H_

#ifdef WIN32
#pragma warning(disable : 4786)
#endif

#include <Basic.h>
#include <Point.h>
#include <Vector.h>
using namespace mathtool;

#include <list>
#include <cassert>
#include <vector>
using namespace std;

#include <limits.h>
#include <float.h>

#include "simple_svg_1.0.0.h"

typedef unsigned int uint;


//
//a triangle
//
struct triangle
{
    uint v[3]; // id to the vertices
};


//
// Extra information for Vertex of polygon
//

class c_BPC; //defined in bpc.h, a class for bridge, pocket and concavity
#include "diagonal2.h"
#include "cutset.h"

class ply_vertex;
class c_ply;
class c_polygon;
struct ply_vertex_extra
{

    ply_vertex_extra(){ 
		concavity_bpc=NULL; concavity=0; flag=0; other_v=NULL; isHoleDiag=false;
	    isHoleFeaturePnt=false; parentPly=NULL; parentPolygon=NULL; concavity_hole=NULL;featureValue=0;holeCutConcavity = 0;accLen=0;}

    void reInit(){
    	concavity_bpc=NULL; concavity=0; flag=0; other_v=NULL; isHoleDiag=false;
    	isHoleFeaturePnt=false; parentPly=NULL; parentPolygon=NULL; concavity_hole=NULL;featureValue=0;holeCutConcavity = 0;accLen=0;
    }

    //is this vertex a pocket minimum
    bool isPM() const { return (concavity_bpc!=NULL && concavity>0) || (concavity_hole!=NULL && concavity>0); }

	//is this vertex a feature point on the hole
	//bool isHolePM() const { return concavity_hole!=NULL && concavity>0; }

	ply_vertex * getDihedralPre();  //get previous vertex for estimating the dihedral angle
    ply_vertex * getDihedralNext(); //get next vertex for estimating the dihedral angle

    void setConcavity_hole(c_ply* thole)
	{
    	concavity_hole = thole;
	}
    const c_ply* getConcavity_hole() const { return concavity_hole; }


    c_BPC * concavity_bpc; //this bpc defines the concavity of this vertex
	


    float   concavity;
    uint    flag;
    vector<c_diagonal> diagonals;
    vector<c_cutset> cutsets;

    ply_vertex * other_v; //copied vertex in original/simplified polygon

    static uint getFlagID(){ static uint id=1; return id++; }
	//is hole diagonal vertex
	bool isHoleDiag;
	bool isHoleFeaturePnt;
	c_ply* parentPly;
	c_polygon* parentPolygon;

	float featureValue;//for clustering diagonals

	float accLen;		//accumulated length from start vertex... Added by Zhonghua 04/06/2013
	float holeCutConcavity; //if the vertex is an end point of a cut for its hole. add by guilin 04/09/2013

private:
	//add by Guilin
	//this defines whether this point is on a hole boundary
	c_ply* concavity_hole;

};


//
// Vertex of polygon
//
class ply_vertex
{
public:

    ///////////////////////////////////////////////////////////////////////////
    ply_vertex(){ init(); }
    ply_vertex( const Point2d& p ){ pos=p; init(); }
    virtual ~ply_vertex();
    void setNext(ply_vertex * n){next=n; if(n!=NULL) n->pre=this; }
    void setPre(ply_vertex * n){pre=n; if(n!=NULL) n->next=this; }
    void computeExtraInfo();

    //negate the vertex
    void negate();

    //reverse the order
    void reverse();

    //copy
    void copy(ply_vertex * other);

    ///////////////////////////////////////////////////////////////////////////
    void setPos(const Point2d& p) { pos=p; }
    virtual const Point2d& getPos() const { return pos; }
	float distanceTo(ply_vertex* other);

    void translate(const Vector2d& v){ pos=pos+v; }

    void rotate(double r);

    virtual ply_vertex * getNext() const { return next; }
    virtual ply_vertex * getPre() const { return pre; }

    const Vector2d& getNormal() const { return normal; }
    bool isReflex() const { return reflex; }

    //get extra information
    uint getVID() const { return vid; }
    void setVID(uint id) {vid=id;}
    ply_vertex_extra& getExtra() { return extra; }
    const ply_vertex_extra& getExtra() const { return extra; }



    //extra info for decomposition
    ply_vertex_extra extra;

private:

    void init(){
        next=pre=NULL;
        reflex=false;
        vid=UINT_MAX;
		extra.isHoleDiag=false;
    }

    //basic info
    Point2d pos;       //position
    ply_vertex * next; //next vertex in the polygon
    ply_vertex * pre;  //previous vertex in the polygon
    Vector2d normal;   //normal, the segment normal from this v to the next.
    bool reflex;
    uint vid;

};

//
// Polygon chain
//
class c_ply{
public:

    enum POLYTYPE { UNKNOWN, PIN, POUT };

    ///////////////////////////////////////////////////////////////////////////
    c_ply(POLYTYPE t){ head=tail=NULL; type=t; radius=-1; area=-FLT_MAX; canBeIgnored = false;}

    ///////////////////////////////////////////////////////////////////////////
    void copy(const c_ply& ply); //copy from the other ply
    void destroy();

    ///////////////////////////////////////////////////////////////////////////
    // create c_ply
    void beginPoly();
    ply_vertex * addVertex( double x, double y, bool remove_duplicate=false );
    ply_vertex * addVertex( ply_vertex * v );
    void endPoly(bool remove_duplicate=false);

    ///////////////////////////////////////////////////////////////////////////
    void negate();
    void reverse(); //reverse vertex order
    void reverseType(); //revise pin to pout or vice versa

    ///////////////////////////////////////////////////////////////////////////
    void translate(const Vector2d& v);
    void rotate(double radius);
    void scale(float f);

    ///////////////////////////////////////////////////////////////////////////
    //
    Point2d findEnclosedPt(); //find a point that is enclosed by this polychain

    //triangulate the polygon
    void triangulate(vector<triangle>& tris);

    ///////////////////////////////////////////////////////////////////////////
    // Access
    ply_vertex * getHead() const { return head; }
    POLYTYPE getType() const { return type; }
    void set(POLYTYPE t,ply_vertex * h){
        type=t; head=h;
        if(h!=NULL){ tail=h->getPre(); }
        else{ tail=NULL; }
    }
    int getSize() {
        if(all.empty()) build_all();
        return all.size();
    }

    ply_vertex * operator[](unsigned int id){
        if(all.empty()) build_all();
        return all[id];
    }

    ///////////////////////////////////////////////////////////////////////////
    // additional functions
    const Point2d& getCenter();

    ///////////////////////////////////////////////////////////////////////////
    //compute the Radius of the poly chain
    float getRadius();

    //area
    float getArea();

    //check if a point is enclosed
    //the behavior is unknown if pt is on the boundary of the polygon
    bool enclosed(const Point2d& pt);

    //check if convex
    bool is_convex() const;

    //delete a vertex
    void delete_vertex(ply_vertex * p);

    ///////////////////////////////////////////////////////////////////////////
    // Operator
    //check if give poly line is the same as this
    bool operator==(const c_ply& other ) const{ return other.head==head; }
    friend istream& operator>>( istream&, c_ply& );
    friend ostream& operator<<( ostream&, c_ply& );

    bool canBeIgnored;

    ///////////////////////////////////////////////////////////////////////////
    bool doInit(); /*return # of vertice in this poly*/

    //build elements in vector<ply_vertex*> all
    void build_all();

    void re_triangulate();

protected:



private:

    ply_vertex * head; //the head of vertex list
    ply_vertex * tail; //end of the vertex list

    vector<ply_vertex*> all; //all vertices

    //additional info
    Point2d center;
    float radius;
    float area;

    //In, out or unknown.
    POLYTYPE type;

    //triangulation
    vector<triangle> triangulation; //catched triangulation, calculated by triangulate
};


//a c_plylist is a list of c_ply
class c_plylist : public list<c_ply>
{
    friend ostream& operator<<( ostream&, c_plylist& );
    friend istream& operator>>( istream&, c_plylist& );

public:

    c_plylist()
    {
        box[0]=box[1]=box[2]=box[3]=0;
        is_buildboxandcenter_called=false;
    }

    void negate();
    void translate(const Vector2d& v);
    void rotate(double r);

    //access
    void buildBoxAndCenter();
    double * getBBox() { assert(is_buildboxandcenter_called); return box; }
    const Point2d& getCenter() { assert(is_buildboxandcenter_called); return center; }

protected:

    Point2d center;
    double box[4];

private:

    bool is_buildboxandcenter_called;
};

//
// a c_polygon is a restricted kind of c_plylist
// this defines a simple polygon so that
// the first element much be a POUT c_ply and
// the rest ply lines are a list of holes
//
class c_polygon : public c_plylist
{
public:

    c_polygon() { area=0; bCnveXEnough = false;}

    void push_back(const c_ply& ply)
    {
        c_plylist::push_back(ply);
        all.clear();
        build_all();
    }

    bool valid(); //check if this is a valid polygon

    //copy from the given polygon
    void copy(const c_polygon& other);

    //triangulate the polygon
    void triangulate(vector<triangle>& tris);

    list<c_polygon> split();

    void reverse(); //reverse the vertex order (not the list order)

    void scale(float factor);

    void normalize();

    //access the vertices of the polygon as an array
    uint getSize()
    {
        if(all.empty()) build_all();
        return all.size();
    }

    //get number of vertices
    uint getSize() const
    {
        assert(all.empty()==false);
        return all.size();
    }


    ply_vertex * operator[](unsigned int id){
        if(all.empty()) build_all();
        return all[id];
    }

    ply_vertex * operator[](unsigned int id) const {
        assert(all.empty()==false);
        return all[id];
    }

	//convert this polygon to SVG polygon
	void toSVG(svg::Polygon & svg_poly);

    double getArea();

    //destroy
    void destroy();

    //check if a point is enclosed
    //the behavior is unknown if pt is on the boundary of the polygon
    bool enclosed(const Point2d& pt);

    //find a point inside the polygon
    Point2d findEnclosedPt();

    bool is_convex() const;


    void build_all();

    bool bCnveXEnough;

private:


    vector<ply_vertex*> all; //all vertices

    //triangulation
    vector<triangle> triangulation; //catched triangulation, calculated by triangulate

    float area;
};


//global singleton
c_polygon& getP();
c_polygon& getQ();

#endif //_MKSUM2D_POLYGON_H_



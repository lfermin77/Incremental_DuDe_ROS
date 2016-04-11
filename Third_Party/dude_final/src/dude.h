#pragma once

#include "polygon.h"
#include "diagonal2.h"
#include "bpc.h"
#include "cutset.h"
#include "holediag.h"

#include <map>
using namespace std;

static uint TMPFLAG = 9999999;

class c_dude
{
public:
	c_dude(){}

	void clear();

    void build(c_polygon& P, double tau, bool isInitPolygon);

    //isDiagonal check if a vertex pair if a diagonal in given "polygon"
    bool isDiagonal(/*const*/ ply_vertex* s, /*const*/  ply_vertex * t, c_polygon& polygon, c_polygon& initPolygon);

    //
    // Access functions
    //
    void addDiagonal(ply_vertex * s, ply_vertex * t, c_polygon& polygon);

    const vector<c_diagonal>& getDiagonals() const {return m_diagonals;}

    const c_polygon& getReducedPolygon() const { return m_reduced_P; }
    const c_polygon& getSimplifiedPolygon() const { return m_simpilifed_P; }
    const list<c_BPC *>& getBPS() const { return m_bpcs; }

    const vector<c_diagonal>& getFinalCuts() const { return m_cuts; }



protected:

    //void decompose_hole(c_ply& hole, list<c_BPC *>& bpcs);
	void decompose_hole(c_ply& hole, list<ply_vertex*>& holePMPnts,list<ply_vertex*>&holeFeatPnt, list<c_BPC *>& bpcs);

    void decompose_bridge(c_BPC * bpc);

    //
    //create simplified and reduced polygons
    //i.e. m_reduced_P and m_simpilifed_P
    //
    void create_simplified_and_reduced(c_polygon& P);

    //keep K best diagonals and remove the rest
    void keepKBest(list<ply_vertex*>& pms, vector<c_diagonal>& cuts);

    //
    //keep diagonals that have end points with at least tau concavity
    //note: tau is scaled by the radius of P
    //
    //void keepTauConcavity(float tau);
    
	//create BPCs from the input polygon
	void buildBPC(c_polygon& P);
	void buildBPC_from_hole(c_ply& P);
	void buildBPC_from_ext(c_ply& P);

    //build simplied ply based on the active flag set in build function
    void buildSimpliedPly(/*const */  c_ply& from,c_ply& to);

    //build mutually exclusive cut sets for each pocket minimum
    void buildCutSets(list<ply_vertex*>& pms);

    //is the segment uv breaking the pocket of v and u?
    bool is_bridge_breaking(const ply_vertex * v, const ply_vertex * u);

    //add diagonal between pocket minia
//    void extractPocketDiagonals(list<c_BPC *>& bpcs);
    
    //create a ply from bridge
    c_ply create_ply_from_bridge(ply_vertex * s, ply_vertex * e);

   // pair<ply_vertex*,ply_vertex*> findHoleCW(c_ply& hole);

    void collectPMs(list<c_BPC *>& bpcs);

public:

	list<c_BPC *> m_bpcs;
	
    c_polygon m_reduced_P;    // this is a polygon with the subset of boundaries of P
    c_polygon m_simpilifed_P; // this is a polygon with only tips and concavities

//    map<const ply_vertex*, ply_vertex*> simp2input; //mapping vertices in simplified polygon to that of input polygon

    uint m_marking_flag;

    list<ply_vertex *> m_PMs; // all pocket minima

    vector<c_cutset>   m_cutsets;   //all cutsets generated from bpcs in build function

    vector<c_diagonal> m_diagonals; //diagonals are created from CDT

    vector<c_diagonal> m_cuts; //final cuts

	double m_tau;

	//added by Guilin
	list<ply_vertex *> hole_PMs;//to record the pocket minimum points of the hole
	list<ply_vertex *> holeFeatPnts;//to record the feature points in the hole
	list<ply_vertex *> convexFeatPnts;//to record the features points on the convex of the extra-boundary
	list<ply_vertex *> poutFeatPnts;//to record the feature points of the whole extra-boundary,
								//that is to combine the pocket features points and convex feature points
	list<ply_vertex*> totalFeatPnts;//all the feature points of this polygon
	list<ply_vertex*> combine_PMs;//combine the extra-boundary and hole boundary feature points
	//list<ply_vertex*> holeAxisPnts;
	


	vector<ply_vertex*> bpcStartPnts;
	vector<ply_vertex*> bpcEndPnts;


	vector<c_diagonal> holeCutDiagonals;

	//combine the extra-boundary pocket minimums and hole minimums
	void combineMinimumPnts(list<ply_vertex*>& extraPMs,list<ply_vertex*>& holePMs,list<ply_vertex*>& totalPMs);
	void collectOutBoundFeatPnts();
	void collectTotalFeatPnts();

	//////////////////////////////////////////////////////////////////////////
	//for the modification of the simplified polygon

	//functional method
	//convert the vertex list to c_ply
	//c_ply list2Ply(list<ply_vertex*> & vertList);
	c_plyline bpc2polyline( const list<ply_vertex*>& vlist );


	//check whether the curVert is contained in bcsStartPnts,
	//if yes, return the index; if no, return -1;
	int checkStartPoint(ply_vertex* curVert);
	int checkEndPoint(ply_vertex* curVert);

	//collect all the feature points on the extra-boundary
	void collectConvexFeatPnts(c_polygon& P,list<ply_vertex*>& boundFeatPnts);

	//collect the convex outboundary feature points, the outboundary is (nearly) already
	void collectOutConvexFeatPnts(c_ply& plyOut, list<ply_vertex*>& convexFeatPnts);

	//collect the features points in a list
	void collectListFeatPnts(list<ply_vertex*>& vlist,list<ply_vertex*>& resVList);

	uint getDudeFlag() {return TMPFLAG++;}

	//find the feature points and concavity for complex hole
	void findFeatAndConcavityForComplexHole(c_ply& hole,vector<ply_vertex*>& cutPnts,list<ply_vertex*>& tmpHolePMs,list<ply_vertex*>& tmpHoleFeatPnts,double tau, HoleDiagonal* rootHoleCut);

	//resolve the intersected cuts
	void resolveIntersectedCuts(c_polygon& tmpPolygon);

	//refine the pocket minimum inside the pocket/bridge
	//void refinePocketPMs();

};

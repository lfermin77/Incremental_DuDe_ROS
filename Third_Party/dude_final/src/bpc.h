#pragma once
#include "polygon.h"
#include "polyline.h"


class c_BPC //a class for bridge pocket  and concavity
{
public:

    c_BPC();


    //
    // core functions
    //
    bool build(ply_vertex * s, ply_vertex * e);
//    void collectKids(); //collect kids of this BPC by going through the pocket vertices

    bool build_for_hole(ply_vertex * s, ply_vertex * e, bool ordered=false);

	////build for hole, check dude.decompose_hole() function for detailed description
	//bool buildCompelete_for_hole(ply_vertex* s,ply_vertex* e);

    void determineConcavity(float tau, bool bReducePM = false);
    bool isOverlapping(c_BPC * other);
    void reorganize_kids_f();

    //void determineTip();
    void determine_PM(float tau);

    //
    // access functions
    //

    ply_vertex * getSource1() const { return m_source[0]; }
    ply_vertex * getSource2() const { return m_source[1]; }

//    ply_vertex * getConcavity() const { return m_concavity; }
    const list<ply_vertex *>& getConcavities() const { return m_concavities; }
    const list<ply_vertex *>& getBridgeEnds() const { return m_bridge_ends; }

    void addKid(c_BPC * kid){ assert(kid); m_kids.push_back(kid);  kid->m_parent=this;}
    const list<c_BPC *>& getKids() const { return m_kids; }
    c_BPC * getParent() const { return m_parent; }
    const c_BPC * getRoot() const { if(m_parent==NULL) return this; return m_parent->getRoot(); }
//    c_BPC * getLeaf(ply_vertex * v); //get a leaf bpc that has the given vertex as concavity
    int getPocketSize() const { return m_pocketsize; }

    void setNext(c_BPC * bpc) { m_next=bpc; if(bpc!=NULL) bpc->m_pre=this; }
    c_BPC * getNext() const { return m_next; }
    c_BPC * getPre() const { return m_pre; }

    bool isKid(c_BPC * other);

    //compute the area enclosed between p1 and p2
    float bridgeArea(ply_vertex * p1, ply_vertex * p2);

protected:

    float dist2Bridge(const Point2d& p);

    //moved to dude_util.h
    //float dist2Seg(const Point2d& s, const Point2d& t, const Point2d& p);

    //check if the given vertices will form a bridge
    //when return 'b', (p1,p2) is a bridge
    //when return 'r', (p2,p1) is a bridge
    //when return '0', (p2,p1) nor (p1,p2) are bridges
    char checkBridge(ply_vertex * p1, ply_vertex * p2);
    

	void getVerticesExcludingKidPockets_f(list<ply_vertex*>& vlist);


	c_plyline bpc2polyline( const list<ply_vertex*>& vlist );
	
	void insert2tree(c_BPC * kid);

	float dist_in_hierarchy(c_BPC * bpc, const Point2d& p);

	Point2d proj2Seg(const Point2d& s, const Point2d& t, const Point2d& p);

	void getSortedFeatures_f(list<ply_vertex*>& PMs,list<ply_vertex*>& features);



	//create c_ply for a list of vertex
	c_ply list2Ply(list<ply_vertex*> & vertlist);

	c_ply seg2Ply(ply_vertex* vstart,ply_vertex* vend);
private:

    ply_vertex * m_source[2];
    
    list<ply_vertex*> m_concavities;
    list<ply_vertex*> m_bridge_ends;

    int          m_pocketsize;  //number of vertics in this pocket

    //connection at the same level
    //in the order of polygon traversal
    c_BPC * m_next;
    c_BPC * m_pre;

    //connection in a tree
    list<c_BPC *> m_kids;
    c_BPC *       m_best_kid; //the kid that has the largest concavity
    c_BPC *       m_parent;


	//added by Guilin
	float m_tau;
	vector<ply_vertex*> kidStartPnts;
	vector<ply_vertex*> kidEndPnts;
	void initPointList();
	int checkStartPoint(ply_vertex* curVert);
	int checkEndPoint(ply_vertex* curVert);
	void markConvexhllPoints(ply_vertex* s,ply_vertex* e,uint marking_flag);
	//uint getDudeFlag() {return BPCTMPFLAG++;}

	void getConvexApproximate(list<ply_vertex*>& approxList);

	//void determineConcavity1(float tau,bool reducePMs = false);
	void determineConcavity_real(float tau,bool reducePMs = false);
};

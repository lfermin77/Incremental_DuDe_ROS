// $Id: Graph.h,v 1.1.1.1 2004/07/23 16:57:32 neilien Exp $

/////////////////////////////////////////////////////////////////////
/**@file  Graph.h
 *
 * General Description
 *     This set of template classes provides an implementation for 
 *     the graph abstract data type. 
 *
 *     The user must provide the parameter type VERTEX, which is
 *     the data that will be stored in each vertex of the graph.
 *     It is assumed the VERTEX class has the following operators
 *     defined for it: << >> == = 
 *
 *     In addition, for weighted graphs, the user must supply
 *     the parameter type WEIGHT, which is the data that will
 *     be stored in each edge of the graph.
 *
 *     The classes in this hierarchy (so far) include:
 *       o AbstractGraph
 *         o WeightedMultiDiGraph (derived from AbstractGraph)
 *           o WeightedGraph (derived from WeightedMultiDiGraph)
 *
 *     Graphs are represented by adjacency lists. 
 *     The unique vertex IDs are typedef'ed short ints (which
 *     should be suffient for most graphs).
 *
 **IMPORTANT* 
 *     For convenience, options have been included for dealing either with 
 *     vertex ids (VIDs) or vertex data (VERTEX). The ones dealing with 
 *     VERTEX data are only 'safe' if VERTEX data is unique.
 *
 * Created
 * @date 7/18/98  
 * @author Nancy Amato
 */
/////////////////////////////////////////////////////////////////////

#ifndef Graph_h
#define Graph_h

#ifdef WIN32
#pragma warning( disable : 4018 )
#endif

////////////////////////////////////////////////////////////////////////////////////////////
//include standard headers
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>   
#include <list>   
#include <vector>
#include <deque>  
#include <stack>

using namespace std;

#ifndef VID
typedef int VID;
#endif

#ifndef EID
typedef short EID;
#endif

#ifndef INVALID_VID
#define INVALID_VID    -999
#endif

///WtEdge is the generic structure for edges in weighted graphs
template<class VERTEX, class WEIGHT> class WtEdgeType;
template<class VERTEX, class WEIGHT> class WtVertexType;

/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          WtEdgeType<VERTEX,WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////


/**WtEdge is the generic structure for edges in weighted graphs.
  *Its data includes:
  * -# the unique_id of the *second* endpoint of the edge (the 
  *    first endpounsigned int is implicit due to the adjacency list rep) 
  * -# the edge WEIGHT
  *
  * Has friend class WtVertexType<VERTEX,WEIGHT>
  */

/////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class WtEdgeType {

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Friend info
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    friend class WtVertexType<VERTEX,WEIGHT>;

public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
       ///Do nothing
       WtEdgeType();
       /**Init data member by given parameters.
         *@param VID the id for the second endpoint
         *@param WEIGHT the weight for this edge.
         */
       WtEdgeType(VID, WEIGHT);
       ///Do nothing
      ~WtEdgeType();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods. Use these method to read in/write out internal state*/
    //@{
        ///Out put Id of the second endpoint and weight to the standard output
        void DisplayEdge() const;
        ///Out put Id of the second endpoint and weight to the given output stream
        void WriteEdge(ostream&) const;
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**unique_id of the *second* endpoint of the edge.
      *start vertex (v1) is implicit in adj list rep.
      */
    VID     vertex2id;

    WEIGHT  weight; ///< Weight of this edge

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          WtVertexType<VERTEX,WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
/**      WtVertexType is the generic structure for vertices in weighted graphs.
  *      Its data includes an edgelist for the vertex, and its methods 
  *      are responsible for adding/removing edges and weights. 
  *
  *      Is friend of class WtEdgeType<VERTEX,WEIGHT>.
  */
template<class VERTEX, class WEIGHT>
class WtVertexType {  

public:

  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
      ///Do nothing
      WtVertexType();

      ///Init data member using given parameters
      WtVertexType(VERTEX&, VID);       // don't reserve space for edges

      /**Init data member using given parameters.
        *@param int how many edges are going to be reserved.
        */
      WtVertexType(VERTEX&, VID, int);  // 'reserve' space for edges

      ///Do nothing
      ~WtVertexType();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  //===================================================================
  /**@name  Access Methods*/
  //===================================================================
  //@{

      /**Adding a edge by given id for the second endpoint and the weight of 
        *this new edge.
        */
      void AddEdge(VID, WEIGHT); 

      /**Adding a edge by given id for the second endpoint and the weight of 
        *this new edge. Check if the edge is already added, if yes, do nothing.
        */
      void AddEdgewCheck(VID, WEIGHT); 

      /**Adding a predecessor edge by given id for the first endpoint and the weight of 
        *this new edge.
        */
    void AddPredecessorEdge(VID, WEIGHT);

      /**Delete edge(s) whose endpounsigned int is given.
        *@param VID the second endpoint of edge.
        *@param int how many edges with given endpoint will be delete.
        *if this parameter is -l. All edges with given endpoint will be delete
        */


      int  DeleteXEdges(VID, int); 
      /**Delete edge(s) whose endpoint and wight are the same as parameters.
        *@param VID the second endpoint of edge.
        *@param WEIGHT the edge with this weight will be deleted.
        *@param int how many edges with given endpoint will be delete.
        *if this parameter is -l. All edges with given endpoint will be delete
        */
      int  DeleteXEdges(VID, WEIGHT, int); 

      /**Check if any edge having this given vid.
        *return true if this kind of edge exists in edgelist.
        */
      bool IsEdge(VID) const ; 

      /**Check if any edge having this given vid and return the address of this edge.
        *return true if this kind of edge exists in edgelist.
        *@param WtEdge** the edge with given VID. If there are more
        *than one edges, this will be the first one found in the list.
        */
      bool IsEdge(VID, const WtEdge**) const;

      /**Check if any edge having this given vid and weight 
        *the address of this edge  will be returned.
        *return true if this kind of edge exists in edgelist.
        *@param WEIGHT qualifier
        *@param WtEdge** the edge with given VID. If there are more
        *than one edges, this will be the first one found in the list.
        */
      bool IsEdge(VID, WEIGHT, const WtEdge** _ei) const;

      //////////////////////////////////////////////////////////////////////////////////////////
      //
      //    Getting Data & Statistics
      //
          //////////////////////////////////////////////////////////////////////////////////////////

      ///Get User specified data stored in this vertex.
      VERTEX  GetData() const; 
      ///Get VID of this instance of WtVertexType.
      int  GetVID() const;
      ///Number of edges connected
      int  GetEdgeCount() const;
      /**Get weight of edge whose second endpoint has VID
        *@param VID the id for looking for edge.
        *@return the first edge having this id was returned.
        */
      WEIGHT  GetEdgeWeight(VID) const; 

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name I/O methods*/
  //@{
        /**Output information about this vertex to standard output.
          *Print out vertex id, user specified data, and every edge.
          *@note this print out some kind of human readable data.
          *@see WriteEdgelist
          */
        void DisplayEdgelist() const;

       /**Output information about this vertex to given output stream.
          *Print out vertex id, user specified data, and every edge.
          *@note this print out some kind of NON-human-readable data.
          *format: VID VERTEX #edges VID WEIGHT VID WEIGHT ... 
          *@see DisplayEdgelist
          */
        void WriteEdgelist(ostream&) const;
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Utility Stuff
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /**@name  Utility Stuff*/
  //===================================================================
  //@{

    //Modified for VC
    /**This seems VC is not be able to use vector<>::iterator directly without
      *"using namespace std;". how every this causes bigger problem.
      *therefore, one more typedef is used here
      */
    typedef  vector< WtEdge > WtEdge_VECTOR;
    typedef  typename WtEdge_VECTOR::iterator EI;
    typedef  typename WtEdge_VECTOR::const_iterator CEI;
    typedef  typename WtEdge_VECTOR::reverse_iterator REI;
    typedef  typename WtEdge_VECTOR::const_reverse_iterator CREI;

    typedef  vector< pair<VID, WEIGHT> > PAIR_VECTOR;
    typedef  typename PAIR_VECTOR::iterator PEI;
    typedef  typename PAIR_VECTOR::const_iterator CPEI;  
    typedef  typename PAIR_VECTOR::reverse_iterator RPEI;
    typedef  typename PAIR_VECTOR::const_reverse_iterator CRPEI;

    // Predicates

    /**check if _eid is an edge in the list, edgelist.
      *@return WtEdge which has this ID
      */
    WtEdge* my_find_EID1_eq(const EID _eid) const; 

    /**check if _eid is an edge in the list starting from _start to _end.
      *@return WtEdge which has this ID
      */
    WtEdge* my_find_EID2_eq(const WtEdge* _start, const WtEdge* _end, const EID _eid) const; 

    /**check if any edge in the list, edgelist, has weight, _wt.
      *@return WtEdge which has this weight
      */
    WtEdge* my_find_EWT_eq(const WEIGHT _wt) const; 

    /**check if any edge in the list, edgelist, has weight, _wtpair.second
      *and ID, _wtpair.first
      *@return WtEdge which has this weight
      */
    WtEdge* my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair) const;
    


  //@}

  // NMA: predicates below don't work with sgi/CC, but do for sun/g++
  //class EID_eq;
  //class EWT_eq;
  //class EIDWT_eq;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  VERTEX data; ///<Vertex Data e.g., cfg, data here...
  VID    vid;  ///< This vertex's unique id (not nec. index)
  vector< WtEdge > edgelist; ///< Adj list rep of graph
  vector< WtEdge > predecessors; ///< call WeightedMultiDiGraph::SetPredecessors() to initialize
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          AbstractGraph<VERTEX> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////

/**   Abstract base class for all graphs.
  *   It is an abstract class (contains pure virtual functions)
  *   and serves as the interface to the derived classes
  *   representing various types of graphs (e.g., undirected,
  *   directed, unweighted, weighted, or multigraphs).
  */

template<class VERTEX> 
class AbstractGraph {

public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

       ///Constrcutor. Set every thing to zero, NULL, and false.
       AbstractGraph();

       /**Constrcutor with reservation.
         *Set reserveEdgesPerVertex to the given value and 
         *set other data members to zero, NULL, and false. 
         *@param int number of edges per vertex ('reserve' space)
         */
       AbstractGraph(int);

       ///Do nothing
       ~AbstractGraph();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods*/
    //@{

       ///////////////////////////////////////////////////////////////////////////////////////////
       //
       // Adding & Finding & Deleting Vertices
       //
       ///////////////////////////////////////////////////////////////////////////////////////////

       ///Abstract
       virtual VID  AddVertex(VERTEX&) = 0;
       ///Abstract
       virtual VID  AddVertex(vector<VERTEX>&) = 0;
       ///Abstract
       virtual int  DeleteVertex(VID) = 0;
       ///Abstract
       virtual int  DeleteVertex(VERTEX&) = 0;
       ///Abstract
       virtual void DeleteAllEdges(VID) = 0; // delete all incident edges 

       ///////////////////////////////////////////////////////////////////////////////////////////
       //
       // Finding Vertices & Edges
       //
       ///////////////////////////////////////////////////////////////////////////////////////////

       ///Abstract
       virtual bool IsVertex(VID) const = 0;
       ///Abstract
       virtual bool IsEdge(VID, VID) const = 0; 
       ///Abstract
       virtual bool IsVertex(VERTEX&) const = 0;
       ///Abstract
       virtual bool IsEdge(VERTEX&, VERTEX&) const  = 0; 

       ///////////////////////////////////////////////////////////////////////////////////////////
       //
       // Getting Data & Statistics
       //
       ///////////////////////////////////////////////////////////////////////////////////////////

       ///Get number of vertices are in this graph
       int  GetVertexCount() const;
       ///Set number of vertices are in this graph
       int  GetEdgeCount() const;
       ///Get vertIDs
       VID  GetNextVID() const;

       ///////////////////////////////////////////////////////////////////////////////////////////
       //
       //Modifying Data
       //
       ///////////////////////////////////////////////////////////////////////////////////////////

        ///To define start number of identifier
        void SetStartID(VID);
        ///Set number of vertices are in this graph
        void SetnumVerts(int);
        ///Set number of edges are in this graph
        void SetnumEdges(int);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods*/
    //@{

       ///Abstract
       virtual void DisplayGraph() const = 0; 
       ///Abstract
       virtual void DisplayVertexAndEdgelist(VID) const = 0; 

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

   VID vertIDs;  ///< used to give each vertex unique identifier     
   int numVerts; ///<How many vertices are in this graph
   int numEdges; ///<How many edges are in this graph   
   int reserveEdgesPerVertex; ///< used to 'reserve' space in edgelist

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          WeightedMultiDiGraph<class VERTEX, class WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////

class dfsinfo;

/**Derived from AbstractGraph<VERTEX,WEIGHT>.
  *
  *The WeightedMultiDiGraph class:
  * -# is a *directed* weighted graph
  * -# allows multiple vertices with the same VERTEX data
  * -# allows multiple (v1id,v2id) edges with *different* WEIGHTs
  * -# allows multiple (v1id,v2id) edges with *same* WEIGHT
  *
  *The graph is represented by an adjacency list structure.
  *
  */

template<class VERTEX, class WEIGHT>
class WeightedMultiDiGraph : public AbstractGraph<VERTEX> {
public:
  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

       /**Constrcutor. Do nothing.
         *@note this constrcutor didn't  reserve any space for verts or edges
         */
       WeightedMultiDiGraph();

       /**Constrcutor. 'reserve' space for vertices.
         *@param int how many vertices will be reserved.
         */
       WeightedMultiDiGraph(int);

       /**Constrcutor. 'reserve' space for vertices.
         *@param first_int how many vertices will be reserved.
         *@param first_int how many edges will be reserved.
         */
       WeightedMultiDiGraph(int,int);

       /**Destrcutor. Do nothing.
         */
       ~WeightedMultiDiGraph();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Vertices
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Vertices*/
   //@{

       /**Add a new (isolated) vertex to graph.
         *A new VID for this new vertex is created automaticallt.
         *@param VERTEX& the user specified data that will be stored in this 
         *new vertex
         *@return vertex id (not nec. index)
         */
       virtual VID  AddVertex(VERTEX&);

       /**Add a list of new (isolated) vertices to graph.
         *BNw VIDs for new vertices are created automaticallt.
         *@param vector<VERTEX>& the user specified data that will be stored in 
         *new vertices
         *@return vertex id (not nec. index) for the first created vertex.
         */
       virtual VID  AddVertex(vector<VERTEX>&);

       /**Delete a vertex according to ID of vertex.
         *Not only this vertex will be delete, but also 
         *those edges connected to this vertex will be deleted.
         *@param VID the id of the vertex which will be killed.
         *@see DeleteAllEdgesToV
         */
       int  DeleteVertex(VID);
       
       /**Delete a vertex according to user data.
         *Not only this vertex will be delete, but also 
         *those edges connected to this vertex will be deleted.
         *@param VERTEX& user data of the vertex which will be killed.
         *@see DeleteAllEdgesToV
         *@note if there are more than one vertex share same user data
         *than the first found in the list will be deleted
         */
       int  DeleteVertex(VERTEX&);

       /**Remove all vertices added to this graph and set every thing 
         *to zero, NULL, and false.
         *@return 0
         */
       int  EraseGraph(); 
   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Edges*/
   //@{
        
       /**Add two edges, from the vid1 to vid2 vertex and 
         *from the vid2 to vid1 vertex.
         *@param vid1 the id for the first vertex
         *@param vid2 the id for the second vertex
         *@note pair.first is weight for vid1->vid2 and pair.second
         *is weight for vid2->vid1
         *@note vid1 and vid2 should be added in graph before, otherwise
         *1 will be returned.
         *@return 0 if vid1 and vid2 are found in this graph and new edge
         *is created correctly
         */
       virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);

       /**Add an edge, from the v1 to v2 vertex with weight.
         *@param vid1 the id for the first vertex
         *@param vid2 the id for the second vertex
         *@param WEIGHT the weight for this new edge
         *@note vid1 and v2 should be added in graph before, otherwise
         *1 will be returned.
         *@return 0 if vid1 and vid2 are found in this graph and new edge
         *is created correctly
         */
       virtual int  AddEdge(VID, VID, WEIGHT);

       /**Add two edges, from the v1 to v2 vertex and 
         *from the v2 to v1 vertex.
         *@param v1 the user data for the first vertex
         *@param v2 the user data for the second vertex
         *@note pair.first is weight for v1->v2 and pair.second
         *is weight for v2->v1
         *@note v1 and v2 should be found in graph, otherwise
         *1 will be returned. if there are more than one vertex
         *constain v1 or v2, then the first one found in the graph will
         *be used.
         *@return 0 if v1 and v2 are found in this graph and new edge
         *is created correctly.
         */
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);

       /**Add an edge, from the v1 to v2 vertex with given weight.
         *@param v1 the user data for the first vertex
         *@param v2 the user data for the second vertex
         *@param WEIGHT the weight for this new edge
         *@note v1 and v2 should be found in graph, otherwise
         *1 will be returned. if there are more than one vertex
         *constain v1 or v2, then the first one found in the graph will
         *be used.
         *@return 0 if v1 and v2 are found in this graph and new edge
         *is created correctly.
         */
       virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT);  

       /**Add edge v1->v2, and v2->v1 to graph with same weight.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *@return 1 if v1 and/or v2 are not in graph.
         *@return 1 if v1 and v2 have been connected.
         *@note enven 2 edges are created, but they are counted as one edge.
     *This is here only for DijkstraSSSP(VID) const, which reqires 
     *this function in
     * sssptree.AddEdge( tmp1, tmp, u.dist);
     *where u.dist is a double type
         */
       virtual int  AddEdge(VERTEX&, VERTEX&, double);

       /**Add a predecessor edge, from the v2 to v1 vertex with weight.
         *@param vid1 the id for the first vertex
         *@param vid2 the id for the second vertex
         *@param WEIGHT the weight for this new edge
         *@note vid1 and v2 should be added in graph before, otherwise
         *1 will be returned.
         *@return 0 if vid1 and vid2 are found in this graph and new edge
         *is created correctly
         */

      virtual int  AddPredecessorEdge(VID, VID, WEIGHT);

       /**Delete n edges from vid1 to vid2.
         *Delete edges starting from vid1 to vid2.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@note vid1 an vid2 should be in this graph
         *@return 1 if vid1 and/or vid2 are not found. 0 if ok.
         */
       virtual int  DeleteEdge(VID, VID, int n=-1); // default, delete all

       /**Delete n edges from v1 to v2.
         *Delete edges starting from v1 to v2.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@note v1 an v2 should be in this graph
         *@return 1 if v1 and/or v2 are not found. 0 if ok.
         */
       virtual int  DeleteEdge(VERTEX&,VERTEX&, int n=-1); //default, delete all

       /**Delete n edges from vid1 to vid2 of specified weight.
         *Delete edges starting from vid1 to vid2 of specified weight.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@param WEIGHT the edges of this weught will be deleted
         *@note vid1 an vid2 should be in this graph
         *@return 1 if vid1 and/or vid2 are not found. 0 if ok.
         */
       virtual int  DeleteWtEdge(VID, VID, WEIGHT, int n=-1); // default, delete all

       /**Delete n edges from v1 to v2 of specified weight.
         *Delete edges starting from v1 to v2 of specified weight.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@param WEIGHT the edges of this weught will be deleted
         *@note v1 an v2 should be in this graph
         *@return 1 if v1 and/or v2 are not found. 0 if ok.
         */
       virtual int  DeleteWtEdge(VERTEX&,VERTEX&,WEIGHT, int n=-1); //default, delete all


       /**Delete all edges connected to and connected from vid.
         *This method will delete all edges connected with vid.
         *@note if vid is not in the graph, not thing will happen.
         */
       void DeleteAllEdges(VID);

       /**Delete all edges connected to and connected from v.
         *This method will delete all edges connected with v.
         *Here v is the vertex constains user data in the parameter.
         *if more than one vertices contain this user data, then
         *this method will delete all edges of the first one found 
         *in the graph.
         *@note if vid is not in the graph, not thing will happen.
         */
       void DeleteAllEdges(VERTEX&);
       
   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Path
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Path*/
   //@{

       /**Connect vertices listed in given vector<VID> with uniform wright.
         *If path contains (vid1,vid2,vid3,...), then
         *edges (vid1,vid2), (vid2,vid3), ... will be added to graph.
         *All of these new edges will have same weight WEIGHT.
         *@param WEIGHT the weight for all new edges.
         *@note 1 will be returned if one of vid in the path
         *could not be found in the graph.
         *@return 0 if every thing is fine.
         */
       //virtual int  AddPath( vector<VID>&, WEIGHT);

       /**Connect vertices listed in given vector<VID> with various wright.
         *If path contains (vid0, vid1,vid2,vid3,...), then
         *edges (vid0,vid1), (vid1,vid2), (vid2,vid3), ... will be added to
         *graph with weight vector[0].second, vector[1].second, vector[2].second,..
         *
         *@param vector< pair<VID,WEIGHT> >& a list of vids and weights.
         *@note 1 will be returned if one of vid in the path
         *could not be found in the graph.
         *@return 0 if every thing is fine.
         */
       //virtual int  AddPath( vector< pair<VID,WEIGHT> >& );

       /**Doublely vertices listed in given vector<VID> with various wright.
         *If path contains (vid0, vid1,vid2,vid3,...), then
         *edges (vid0,vid1), (vid1,vid0), (vid1,vid2), (vid2,vid1), 
         *(vid2,vid3), (vid3,vid2) ... will be added to
         *graph with weight vector[0].second.first, vector[0].second.second
         *vector[1].second.first, vector[1].second.second,
         *vector[2].second.first,vector[2].second.second..
         *
         *@param vector< pair<VID, pair<WEIGHT,WEIGHT> > >& a list of vids and weights.
         *@note 1 will be returned if one of vid in the path
         *could not be found in the graph.
         *@return 0 if every thing is fine.
         */
       //virtual int  AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& );

       /**Connect vertices listed in given vector<VID> with uniform wright.
         *If path contains (v1,v2,v3,...), then
         *edges (v1,v2), (v2,v3), ... will be added to graph.
         *All of these new edges will have same weight WEIGHT.
         *
         *@param WEIGHT the weight for all new edges.
         *@note if vi is not found in graph then a new vertex will be added.
         *@return Always 0.
         */
       //virtual int  AddPath( vector<VERTEX>&, WEIGHT); // all edges same weight

       /**Connect vertices listed in given vector<VID> with various wright.
         *If path contains (v0, v1,v2,v3,...), then
         *edges (v0,v1), (v1,v2), (v2,v3), ... will be added to
         *graph with weight vector[0].second, vector[1].second, vector[2].second,..
         *
         *@param vector< pair<VERTEX,WEIGHT> >& a list of user data and weights.
         *@note if vi is not found in graph then a new vertex will be added.
         *@return Always 0.
         */
       //virtual int  AddPath( vector< pair<VERTEX,WEIGHT> >& );

       /**Doublely vertices listed in given vector<V> with various wright.
         *If path contains (v0, v1,v2,v3,...), then
         *edges (v0,v1), (v1,v0), (v1,v2), (v2,v1), 
         *(v2,v3), (v3,v2) ... will be added to
         *graph with weight vector[0].second.first, vector[0].second.second
         *vector[1].second.first, vector[1].second.second,
         *vector[2].second.first,vector[2].second.second..
         *
         *@param vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& a list of user data and weights.
         *@note if vi is not found in graph then a new vertex will be added.
         *@return Always 0.
         */
       //virtual int  AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& );

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Search methods (Finding Vertices & Edges)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Search methods.
     *Finding Vertices & Edges.
     */
   //@{

       /**Check if given vid is a vertex in this graph.
         *@param VID check if any vertex in this graph has this vid 
         *@return true if find one. otherwise false.
         */
       bool IsVertex(VID) const;
       /**Check if any vertex in this graph contains this user data.
         *@param VERTEX& user data 
         *@return true if find one. otherwise false.
         */
       bool IsVertex(VERTEX&) const;

       /**Check if there is any edge connected from vid1 to vid2.
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         */
       bool IsEdge(VID, VID) const;

       /**Check if there is any edge connected from v1 to v2.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         */
       bool IsEdge(VERTEX&, VERTEX&) const;

       /**Check if there is any edge connected from vid1 to vid2 of specified weight.
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         */
       bool IsEdge(VID, VID, WEIGHT) const;

       /**Check if there is any edge connected from v1 to v2 of specified weight.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         */
       bool IsEdge(VERTEX&, VERTEX&, WEIGHT) const;

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access methods (Getting Data & Statistics, global information)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Adding & Deleting Edges.
     *Getting Data & Statistics, global information.
     */
   //@{

       ///Get All VIDs in this graph.
       vector<VID> GetVerticesVID() const;
       /**Get n VIDs starting with VID _vid.
         *@note if _vid is not found in this graph, then a empty vector will be returned,
         *and an error message will be output to standard output.
         */
       vector<VID> GetVerticesVID(VID _vid,int n) const;

       ///Get All user data in this graph.
       vector<VERTEX> GetVerticesData() const;
       /** get n user data starting with VID _vid.
         *@note if _vid is not found in this graph, then a empty vector will be returned,
         *and an error message will be output to standard output.
         */
       vector<VERTEX> GetVerticesData(VID _vid,int n) const;

       /**Get All edges in this graph.
         *@return A edge list. One edge is defined as 2 VIDs and its weight.
         */
       virtual vector< pair<pair<VID,VID>, WEIGHT> > GetEdges() const;
       /**Get All edges and their user data in this graph.
         *@return A edge list. One edge is defined by 2 user data
         *on the two endpoints and its weight.
         */
       virtual vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;

       /**Get user data associated with given VID.
         *If this vid is nit found in graph, then Invalide data 
         *provided by VERTEX::InvalidData() will be returned.
         */
       inline VERTEX  GetData(VID) const;
       /**Get user data associated with VIDs, from _v1id to _v2id.
         *if _v1id and/or _v2id are not found in graph, then
         *an empty vector will be returned.
         *@note _v1id should be smaller than _v2id
         */
 
      ///Get a pointer of User specified data stored in this vertex.
        VERTEX* GetReferenceofData(VID); 

       vector<VERTEX>  GetData(VID _v1id, VID _v2id) const;


       /**Get VID from given user data.
         *If not such vertex is found, then INVALID_VID will
         *be returned.
         */
       VID     GetVID(VERTEX&) const;

       /**Get weight of edge (vid1->vid2).
         *if no such edge WEIGHT::InvalidWeight() will be returned.
         */
       WEIGHT  GetEdgeWeight(VID, VID) const;

       /**Get weight of edge (v1->v2).
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *if no such edge WEIGHT::InvalidWeight() will be returned.
         */
       WEIGHT  GetEdgeWeight(VERTEX&, VERTEX&) const;

      /**Put user data to the specified vertex.
        *If this specified vertex could not be found in graph,
        *then nothing will happen.
        *@see WtVertexType::data
        */
      void PutData(VID, VERTEX);

      /**Get out degree of a given vertex.
        *out degree of a vertex is number of edges
        *that are from this vertex to any other vertex in graph.
        *@return 1 if no this vertex.
        */
       virtual int GetVertexOutDegree(VID) const;

       /**Get VIDs which are connected to by edges going out from
         *specied VID.
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         */
       virtual vector<VID> GetSuccessors(VID) const;

       /**Get VERTEXs which are connected to by edges going out from
         *specied VID.
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         */
       virtual vector<VERTEX> GetSuccessorsDATA(VID) const;

       /**Get VIDs which are connected to by edges going out from
         *specied v.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         */
       virtual vector<VID> GetSuccessors(VERTEX&) const;

       /**Get VERTEXs which are connected to by edges going out from
         *specied v.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         */
       virtual vector<VERTEX> GetSuccessorsDATA(VERTEX&) const;

       /**Initialize predecessors in the data field of Vertex.
         *Predecessors tells client where the edges that 
         *connected to this vertex are from.
         */
       void SetPredecessors();

    //=======================================================
    //The following methods need to call SetPredecessors() first
    //to initialize predecessor vector
    //=======================================================

       /**Get all predecessors of a specied VID as a vector<VID>.
         *@note Need to call SetPredecessors() first to initialize predecessor vector
         *before using this method.
         *
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         *@see SetPredecessors
         */
       virtual vector<VID> GetPredecessors(VID) const;

       /**Get all predecessors of a specied VID as a vector<VERTEX>.
         *@note Need to call SetPredecessors() first to initialize predecessor vector
         *before using this method.
         *
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         *@see SetPredecessors
         */
       virtual vector<VERTEX> GetPredecessorsDATA(VID) const;

       /**Get all predecessors of a specied v as a vector<VID>.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *
         *@note Need to call SetPredecessors() first to initialize predecessor vector
         *before using this method.
         *
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         */
       virtual vector<VID> GetPredecessors(VERTEX&) const;

       /**Get all predecessors of a specied v as a vector<VERTEX>.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *
         *@note Need to call SetPredecessors() first to initialize predecessor vector
         *before using this method.
         *
         *@return empty vector If there is no such vertex found and
         *error message will be output to standard output.
         */
       virtual vector<VERTEX> GetPredecessorsDATA(VERTEX&) const;

       /**Return VIDs of vertices which are sinks.
         *Sink vertex is the vertex that has no outgoing edges (i.e. no successors)
         *This method returns all sinks in the graph.
         */
       vector<VID> GetSinks() const;

       /**Return VIDs of vertices which are sources.
         *Source vertex is the vertex that has no incoming edges (i.e. no predecessors)
         *This method returns all sources in the graph.
     *Need to call SetPredecessors() for the graph before using this method
         */
       vector<VID> GetSources() const;

       //virtual vector< pair< pair<VID,VID>, WEIGHT> > GetIncidentEdges(VID) const;
       //virtual vector< pair< pair<VID,VID>, WEIGHT> > GetIncomingEdges(VID) const;
       //virtual vector< pair< pair<VID,VID>, WEIGHT> > GetOutgoingEdges(VID) const;
  

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Basic Graph Algorithms
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Basic Graph Algorithms*/
   //@{


       //////////////////////////////////////////////////////////////////////////////////////////
       //
       //   BFS
       //
       //////////////////////////////////////////////////////////////////////////////////////////

       /**Breadth First Search,
         *Starting to build tree(s) from given VID vertex.
         *A WeightedMultiDiGraph contains BFS tree(s) will be returned.
         *@return If no VID vertex is found, an empty WeightedMultiDiGraph will
         *be returned. If something wrong during processing, error message will 
         *be output to standard output.
         */
       WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VID) const;

       /**Breadth First Search,
         *Starting to build tree(s) from given vertex with specified user data.
         *A WeightedMultiDiGraph contains BFS tree(s) will be returned.
         *@return If no VID vertex is found, an empty WeightedMultiDiGraph will
         *be returned. If something wrong during processing, error message will 
         *be output to standard output.
         */
       WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VERTEX&) const; 

       /**Breadth First Search,
         *VIDs will be returned instead of a WeightedMultiDiGraph.
         *This function call BFS(VID) to get WeightedMultiDiGraph and
         *@return a list of VIDs by invoking GetVerticesVID().
         */
       vector<VID> BFSVID(VID) const; 

       /**Breadth First Search,
         *VIDs will be returned instead of a WeightedMultiDiGraph.
         *This function call BFS(VERTEX&) to get WeightedMultiDiGraph and
         *@return a list of VIDs by invoking GetVerticesVID().
         */
       vector<VID> BFSVID(VERTEX&) const; 

       /**Find BFS path between 2 specified vertices.
         */
       vector< pair<VERTEX,WEIGHT> > FindPathBFS(VID,VID) const;

       /**Find BFS path between 2 specified vertices.
         *@see FindPathBFS(VID,VID) and IsVertex(VERTEX&)
         */
       vector< pair<VERTEX,WEIGHT> > FindPathBFS(VERTEX&,VERTEX&) const;

       /**Find BFS path between 2 specified vertices. Return the path as a vector<VID>
         *@see FindPathBFS(VID,VID) and IsVertex(VERTEX&)
         */    
       vector< VID > FindVIDPathBFS(VID,VID) const;   

       //////////////////////////////////////////////////////////////////////////////////////////
       //
       //   DFS
       //
       //////////////////////////////////////////////////////////////////////////////////////////

       /**Depth First Search,
         *Find all dfs trees in graph.
         *@return dfs trees.
         */
       vector<WeightedMultiDiGraph<VERTEX,WEIGHT> > DFS() const;

       /**Depth First Search,
         *find a dfs tree in graph, starting from given vertex, VID.
         *@return a dfs tree
         */
       WeightedMultiDiGraph<VERTEX,WEIGHT> DFS (VID&) const;

       /**Depth First Search,
         *find a dfs tree in graph, starting from given vertex of
         *user specified data.
         *@return a dfs tree if everything is ok. Return a empty
         *tree if no such vertex could be find in graph.
         */
       WeightedMultiDiGraph<VERTEX,WEIGHT> DFS (VERTEX&) const;

       /**Depth First Search,
         *VIDs will be returned instead of a WeightedMultiDiGraph.
         *This function call DFS (VID&) to get WeightedMultiDiGraph and
         *return a list of VIDs by invoking GetVerticesVID().
         */
       vector<VID> DFSVID(VID&) const;

       /**Depth First Search,
         *VIDs will be returned instead of a WeightedMultiDiGraph.
         *This function call DFS (VERTEX&) to get WeightedMultiDiGraph and
         *return a list of VIDs by invoking GetVerticesVID().
         */
       vector<VID> DFSVID(VERTEX&) const;

       //////////////////////////////////////////////////////////////////////////////////////////
       //
       //   Dijkstra
       //
       //////////////////////////////////////////////////////////////////////////////////////////

       /**Calculate a Dijkstra sssp tree.
         *returns a sssp tree where edge weights are distances from source
         *i.e., the edge weights are cumulative path lengths.
         *This tree starts from VID and to all vertices which are reachable from this vertex.
         */
       WeightedMultiDiGraph<VERTEX,WEIGHT> DijkstraSSSP(VID) const; //wts=pathlength

       /**Find path of 2 specified vertex in Dijkstra sssp tree.
         *@see DijkstraSSSP and FindPathBFS(VERTEX&,VERTEX&)
         */
       vector< pair<VERTEX,WEIGHT> >  FindPathDijkstra(VID,VID) const; //wts=ewts

       /**Find path of 2 specified vertex in Dijkstra sssp tree.
         *@see FindPathDijkstra(VID,VID)
         */
       vector< pair<VERTEX,WEIGHT> >  FindPathDijkstra(VERTEX&,VERTEX&) const;

       //////////////////////////////////////////////////////////////////////////////////////////
       //
       //   Other
       //
           //////////////////////////////////////////////////////////////////////////////////////////

       /**Check if there is any cycle in this graph.
         *@return true if there is cycle(s) in this graph, otherwise 
         *@see dfs.backedge_vector and aux_DFS
         */
       bool IsCycle() const;

       /**Get each edge that makes a cycle in the graph.
         *@see dfs.backedge_vector and aux_DFS
         */
       vector<pair<VID,VID> > GetBackedge() const;


       /**Topology sort,
         *sorted by finish time.
         *@return a list of vids ordered by finish time.
         *@see aux_DFS, dfs.finish_time, and FinishLate.
         */
       vector<VID> TopologicalSort() const;


   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods*/
    //@{

       /**Display every vertex and edge information in the graph.
         *@see WtVertexType::DisplayEdgelist
         */
       void DisplayGraph() const;

       /**Display every vertex information in the graph.
         *This method calls DisplayVertex(VID) iterately.
         *@see DisplayVertex(VID)
         */
       void DisplayVertices() const; 

       /**Display vertex of VID in the graph.
         *This method outputs VID and user data in this vertex.
         *@note if this VID is not found in the graph, an error
         *message will be print out.
         */
       void DisplayVertex(VID) const; 

       /**Display vertex of VID and its edge information in the graph.
         *@see DisplayEdgelist
         *@note if this VID is not found in the graph, an error
         *message will be print out.
         */
       void DisplayVertexAndEdgelist(VID) const; 

       /**Write graph info to the given output stream.
         *This method outputs numVerts, numEdges, vertIDs
         *to the output stream and calls 
         *WtVertexType::WriteEdgelist for each vertex in 
         *this graph.
         *@see WtVertexType::WriteEdgelist
         */
       void WriteGraph(ostream& _myostream) const;


       /**Write graph info to the file of file name, _filename.
         *This method calls WriteGraph(ostream& _myostream).
         */
       void WriteGraph(const char*  _filename) const; 


       /**Read graph info from the given input stream. Assign vid with given values.
         *Read data which were written by WriteGraph.
         @note Error message will be outputed if something wrong
         *during processing.
         */
       /* use vid from input file as internal vid */    
       void ReadGraph(istream& _myistream);

       /* automatic assign vid to each nodes, need to transform vid in the edgelist */
       void ReadGraphwithAutoVID(istream& _myistream);

       /**Read graph info from the file of file name, _filename.
         *This method calls ReadGraph(istream& _myistream).
         */
       void ReadGraph(const char*  _filename);

       /* automatic assign vid to each nodes, need to transform vid in the edgelist */
       void ReadGraphwithAutoVID(const char*  _filename);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Typedefs
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

   
  ///Modified for VC
  /**This seems VC is not be able to use vector<>::iterator directly without
    *"using namespace std;". how every this causes bigger problem.
    *therefore, one more typedef is used here
    */
   typedef vector< Vertex > VERTEX_VECTOR;
   typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
   typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
   typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
   typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator

   typedef vector< WtEdge > WtEdge_VECTOR;
   typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
   typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
   typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
   typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: Adding & Deleting Vertices & Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  /**@name Adding & Deleting Vertices & Edges
    */

   //@{

       /**Add a new (isolated) vertex to graph with specifed VID.
         *
         *@param VERTEX& the user specified data that will be stored in this 
         *new vertex
         *@return vertex id (not nec. index) which is given by client.
         *@see AddVertex(VERTEX&)
         *@note this method won't create a new id for this new vertex.
         */
       VID  AddVertex(VERTEX&,VID);

       /**Create a new edge from vid to EI->vertex2id.
         *Acutelly, this edgae, EI, has been created, this method
         *just adds this edge to vid's edge list.
         *@return 1 if there is no vertex of given VID in graph.
         *0 is every thing is fine.
         */
       virtual int AddEdge(VID, EI);


       /**Delete All edges connection to VID.
         *If there is no vertex of given VID in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesToV(VID);

       /**Delete All edges connection to v.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *If there is no vertex of given user data in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesToV(VERTEX&); 

       /**Delete All edges going out from VID.
         *If there is no vertex of given VID in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesFromV(VID);

       /**Delete All edges going out from v.
         *Here v is any vertex contains user data in the parameter
         *if there are more than one, then the first will be applied.
         *If there is no vertex of given user data in graph,
         *then nothing will be done.
         */
       virtual void DeleteAllEdgesFromV(VERTEX&); 

   //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: Search methods (Finding Vertices & Edges)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Search methods.
     *Finding Vertices & Edges.
     */
   //@{

       /**Check if given VID is in graph.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VID)
         */
       bool IsVertex(VID, const Vertex**) const;
       /**Check if given v with specified user data is in graph.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VERTEX&)
         */
       bool IsVertex(VERTEX&, const Vertex**) const;


       /**Check if there is any edge connected from vid1 to vid2.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         *@see IsEdge(VID, VID)
         */
       bool IsEdge(VID, VID, const Vertex**, const WtEdge**) const;
       /**Check if there is any edge connected from v1 to v2.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         *@see IsEdge(VERTEX&, VERTEX&)
         */
       bool IsEdge(VERTEX&, VERTEX&, const Vertex**, const WtEdge**) const;

       /**Check if there is any edge connected from vid1 to vid2 of specified weight.
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         */
       bool IsEdge(VID, VID, WEIGHT, const Vertex**, const WtEdge**) const;

       /**Check if there is any edge connected from v1 to v2 of specified weight.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *If the answer is yes then Vertex** will be the pointer
         *pointing to the vertex where the edge from, and WtEdge**
         *will be the pointer pointing to this edge.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         */
       bool IsEdge(VERTEX&, VERTEX&, WEIGHT, const Vertex**, const WtEdge**) const;

       /**Find vertex of _vid in graph.
         *If no such vertex, then v.end() will be returned.
         */
       Vertex* my_find_VID_eq(const VID _vid) const;

       /**Find vertex of user data _v in graph.
         *If no such vertex, then v.end() will be returned.
         *@note this method compares every vertex in graph.
         */
       Vertex* my_find_VDATA_eq(const VERTEX& _v) const;

       /**Is _v1's VID smaller than _v2's VID?.
         *@return (_v1.vid < _v2.vid )
         */

#ifdef __HP_aCC
       static bool VID_Compare (const Vertex _v1, const Vertex _v2); 
#else 
       static bool VID_Compare (const Vertex& _v1, const Vertex& _v2); 
#endif

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: Basic Graph Algorithms
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Basic Graph Algorithms*/
   //@{

       //////////////////////////////////////////////////////////////////////////////////////////
       //
       //   DFS, Depth First Search Algorithm
       //
       //////////////////////////////////////////////////////////////////////////////////////////

       /**Find all DFS trees in graph.
         *This is a driver, to find all backedges, and topological order of vertices.
         *@see true_DFS, IsCycle, GetBackedge, ad TopologicalSort
         */
       void aux_DFS(dfsinfo&) const;

       /**Core DFS algorithm, start w/ vertex vid
         *@return a dfstree started at vid also return dfsinfo
         *@note iteratively generate DSF tree instread of recursively.
         *@note if VID is not found, then error message will be print out.
         */
       WeightedMultiDiGraph<VERTEX,WEIGHT> true_DFS (VID&,dfsinfo&) const;

       /**Comparing whose finsih time is later than other's.
         *@return (_firstpair.second > _secondpair.second)
         *@see TopologicalSort
         */

#ifdef __HP_aCC
       static bool FinishLate(const pair<VID,int>,const pair<VID,int>); 
#else 
       static bool FinishLate(const pair<VID,int>&,const pair<VID,int>&); 
#endif

       //////////////////////////////////////////////////////////////////////////////////////////
       //
       //   Dijkstra
       //
       //////////////////////////////////////////////////////////////////////////////////////////

       ///Data structure for Dijkstra's algorithm
       class dkinfo {
       public:
         dkinfo() {}
         dkinfo(VID _vid, VID _pvid, double _d) {vid=_vid; predvid=_pvid; dist=_d;}
         VID    vid;
         VID    predvid;
         double dist;
       };

       /**Comparing distances.
         *@return _d1.dist > _d2.dist
         */
#ifdef __HP_aCC
       static bool dkinfo_Compare (const dkinfo _d1, const dkinfo _d2); 
#else 
       static bool dkinfo_Compare (const dkinfo& _d1, const dkinfo& _d2); 
#endif

       //NMA: the following predictates work with stl/sun/g++ but not stl/sgi/CC
       //class VID_eq; 
       //class VDATA_eq;
       //class VID_Compare;

   //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected: DATA
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    vector< Vertex >    v;  ///< vertices (with adj lists)

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data members and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

/////////////////////////////////////////////////////////////////////
//
//
//
//
//          WeightedGraph<VERTEX,WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////


/**The graph is represented by an adjacency list structure.
  *Derived from WeightedMultiDiGraph<VERTEX,WEIGHT>.
  *
  *The WeightedGraph class:
  * -# is an *undirected* weighted graph  (but can have different
  *   weights on the 'forward' and 'back' edges).
  * -# allows multiple vertices with the same VERTEX data 
  * -# does not allow multiple (v1id,v2id) edges (even w/ different WEIGHTs)
  *
  */

template<class VERTEX, class WEIGHT>
class WeightedGraph : public WeightedMultiDiGraph<VERTEX,WEIGHT> {
public:

  ///Vertex Abbreviation of WtVertexType<VERTEX,WEIGHT>
  typedef WtVertexType<VERTEX,WEIGHT> Vertex;

  ///WtEdge Abbreviation of WtEdgeType<VERTEX,WEIGHT>
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

       /**Constrcutor. Do nothing.
         *@note this constrcutor didn't  reserve any space for verts or edges
         *@see WeightedMultiDiGraph::WeightedMultiDiGraph()
         */
       WeightedGraph();

       /**Constrcutor. 'reserve' space for vertices.
         *@param int how many vertices will be reserved.
         *@see WeightedMultiDiGraph::WeightedMultiDiGraph(int)
         */
       WeightedGraph(int);

       /**Constrcutor. 'reserve' space for vertices.
         *@param first_int how many vertices will be reserved.
         *@param first_int how many edges will be reserved.
         *@see WeightedMultiDiGraph::WeightedMultiDiGraph(int,int)
         */
       WeightedGraph(int,int);

       //WeightedGraph(WeightedMultiDiGraph<VERTEX,WEIGHT>); // construct from base  

       /**Destrcutor. Do nothing.
         */
       ~WeightedGraph();

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Adding & Deleting Edges*/
    //@{

       /**Add edge vid1->vid2, and vid2->vid1 to graph with same weight.
         *@return 1 if vid1 and/or vid2 are not in graph.
         *@return 1 if vid1 and vid2 have been connected.
         *@note enven 2 edges are created, but they are counted as one edge.
         */
       virtual int  AddEdge(VID, VID, WEIGHT);

       /**Add edge v1->v2, and v2->v1 to graph with same weight.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *@return 1 if v1 and/or v2 are not in graph.
         *@return 1 if v1 and v2 have been connected.
         *@note enven 2 edges are created, but they are counted as one edge.
         */
       virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT);

       /**Add edge vid1->vid2, and vid2->vid1 to graph with 2 different weights.
         *@return 1 if vid1 and/or vid2 are not in graph.
         *@return 1 if vid1 and vid2 have been connected.
         *@note enven 2 edges are created, but they are counted as one edge.
         */
       virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);

       /**Add edge v1->v2, and v2->v1 to graph with 2 different weights.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *@return 1 if v1 and/or v2 are not in graph.
         *@return 1 if v1 and v2 have been connected.
         *@note enven 2 edges are created, but they are counted as one edge.
         */
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);

       /**Delete all edges from vid1 to vid2 and from vid2 to vid1.
         *No matter what n is given, it always chage n to -1.
         *
         *@param n number of edges will be delete.
         *@note vid1 an vid2 should be in this graph
         *@note enven 2 edges are deleted, but they are counted as one edge.
         *@return 1 if vid1 and/or vid2 are not found. 0 if ok.
         */
       virtual int  DeleteEdge(VID, VID, int _n=-1);

       /**Delete all edges from v1 to v2 and from vid2 to vid1.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if there are more than one, then the first will be applied.
         *
         *No matter what n is given, it always chage n to -1.
         *
         *@param n number of edges will be delete.
         *@note v1 an v2 should be in this graph
         *@note enven 2 edges are deleted, but they are counted as one edge.
         *@return 1 if v1 and/or v2 are not found. 0 if ok.
         */
       virtual int  DeleteEdge(VERTEX&, VERTEX&, int _n=-1);

       /**Delete n edges from vid1 to vid2 and from vid2 to vid1 
         *of specified weight.
         *
         *No matter what n is given, it always chage n to -1.
         *
         *@param WEIGHT the edges of this weught will be deleted
         *@note vid1 an vid2 should be in this graph
         *@note enven 2 edges are deleted, but they are counted as one edge.
         *@return 1 if vid1 and/or vid2 are not found. 0 if ok.
         */
       virtual int  DeleteWtEdge(VID, VID, WEIGHT, int _n=-1);

       /**Delete all edges from v1 to v2 and from vid2 to vid1
         *of specified weight.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if there are more than one, then the first will be applied.
         *
         *No matter what n is given, it always chage n to -1.
         *
         *@param WEIGHT the edges of this weught will be deleted
         *@note v1 an v2 should be in this graph
         *@note enven 2 edges are deleted, but they are counted as one edge.
         *@return 1 if v1 and/or v2 are not found. 0 if ok.
         */
       virtual int  DeleteWtEdge(VERTEX&, VERTEX&, WEIGHT, int _n=-1);

       /**Change edge's (defined be vid1 and vid2) weight to WEIGHT.
         *@param WEIGHT edge's new weight
         *@note this method calls DeleteEdge and AddEdge
         *@return 1 if DeleteEdge returned 1 or AddEdge returned 1.
         *0 if AddEdge returned 0
         *@see DeleteWtEdge(VID, VID, WEIGHT, int _n) and AddEdge(VID, VID, WEIGHT)
         */
       virtual int  ChangeEdgeWeight(VID, VID, WEIGHT);

       /**Change edge's (defined be VERTEX1 and VERTEX2) weight to WEIGHT.
         *@param WEIGHT edge's new weight
         *@note this method calls DeleteEdge and AddEdge
         *@return 1 if DeleteEdge returned 1 or AddEdge returned 1.
         *0 if AddEdge returned 0
         *@see DeleteWtEdge(VERTEX&, VERTEX&, WEIGHT, int _n) and 
         *AddEdge(VERTEX&, VERTEX&, WEIGHT)
         */
       virtual int  ChangeEdgeWeight(VERTEX&, VERTEX&, WEIGHT);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access methods (Getting Data & Statistics, global information)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access methods
      *Getting Data & Statistics, global information
      */
    //@{

       /**Get All edges in this graph.
         *
         *@return A edge list. One edge is defined as 2 VIDs and its weight.
         *@note vid1 is always smaller than vids in returned edge list.
         *This implies this edge is from vid1->vid2. Because this is an undirected
         *graph, the edge vid2->vid1 is also in this graph even if this edge was not
         *included in returned list.
         */
       virtual vector< pair<pair<VID,VID>, WEIGHT> > GetEdges() const;

       /**Get All edges and their user data in this graph.
         *@return A edge list. One edge is defined by 2 user data
         *on the two endpoints and its weight.
         *@note althought v1->v2 was returned, v2->v1 was an edge in graph.
         */
       virtual vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;// vertex information

       /**Get Degree of this specified Vertex.
         *@note For undirected graph, outgoing degree is the same as incoming degree.
         *@retrun Degree if specified VID is found in graph. Otherwise 1 will be returned.
         *@see GetVertexOutDegree
         */
       virtual int GetVertexDegree(VID) const;

       /**Get vetices which are adjacent to this specified vertex.
         *@return a list of VID which are VIDs of those who are next to the specified vertex.
         *If this specified VID is not found, an empty list will be returned and error message
         *will be in standard output.
         *@see GetSuccessors
         */
       virtual vector< VID > GetAdjacentVertices(VID) const;

       /**Get Edges which is incident to this specified vertex.
         *Incident edges are edges around this specified vertex.
         *@return a list of edges which is defined by 2 VIDs of its endpoints and
         *the weight of edge. Empty list will be returned if this specified VID 
         *is not in graph and error message will be in standard output.
         */
       virtual vector< pair<pair<VID,VID>,WEIGHT> > GetIncidentEdges(VID) const;

       /**Get Edges which is incident to this specified vertex and user data associated with
         *these edges.
         *Incident edges are edges around this specified vertex.
         *@return a list of edges which is defined by 2 user data of its endpoints and
         *the weight of edge. Empty list will be returned if this specified VID 
         *is not in graph and error message will be in standard output.
         */
       virtual vector< pair<pair<VERTEX,VERTEX>,WEIGHT> > GetIncidentEdgesVData(VID) const;

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Connected Components Utilities
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Connected Components Utilities*/
    //@{

       /**Check if vid1 and vid2 are in the same component.
         *@return true if vid1 and vid2 are in the same component. Otherwise false
         *will be returned.
         *@see BFS(VID)
         */
       bool IsSameCC(VID,VID) const;

       /**Check if v1 and v2 are in the same component.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if there are more than one, then the first will be applied.
         *
         *@return true if v1 and v2 are in the same component. Otherwise false
         *will be returned.
         *@see BFS(VERTEX&)
         */
       bool IsSameCC(VERTEX&,VERTEX&) const;

       /**Get a list of VIDs which are in the same connected conponent 
         *with this specified vertex.
         *@return if VID is not found in graph then empty list will be returned.
         *@see BFS(VID)
         */
       vector<VID> GetCC(VID) const;


       /**Get a list of user data of vertices which are in the same connected
         *conponent with this specified vertex.
         *@return if VERTEX& is not found in graph then empty list will be returned.
         *@see BFS(VERTEX&)
         */
       vector<VERTEX> GetCC(VERTEX&) const;

       /**Get edges in connected conponent defined this specified vertex.
         *@note This returned edge list might contains duplicate edges. this
         *is when vid1->vid2 was in the list, vid2->vid1 might also be in the list.
         *@return if VID is not found in graph then a empty list will be returned.
         *@see BFS(VID) and GetIncidentEdges
         */
       vector< pair<pair<VID,VID>, WEIGHT> > GetCCEdges(VID) const;

       /**Get edges in connected conponent defined this specified vertex.
         *this specified vertex is defiend by given user data reference.
         *@note This returned edge list might contains duplicate edges. this
         *is when vid1->vid2 was in the list, vid2->vid1 might also be in the list.
         *
         *@return if VID is not found in graph then a empty list will be returned.
         *@see GetCCEdges(VID) and GetVID(VERTEX&)
         */
       vector< pair<pair<VID,VID>, WEIGHT> > GetCCEdges(VERTEX&) const;

       /**Get user data associated with edges in connected conponent 
         *defined this specified vertex.
         *@note This returned edge list might contains duplicate edges. this
         *is when v1->v2 was in the list, v2->v1 might also be in the list.
         *
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if there are more than one, then the first will be applied.
         *
         *@return if VID is not found in graph then a empty list will be returned.
         *@see BFS(VID) and GetIncidentEdgesVData(VID)
         */
       vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetCCEdgesVData(VID) const;

       /**Get user data associated with edges in connected conponent 
         *defined this specified vertex.
         *@note This returned edge list might contains duplicate edges. this
         *is when v1->v2 was in the list, v2->v1 might also be in the list.
         *
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if there are more than one, then the first will be applied.
         *
         *@return if VID is not found in graph then a empty list will be returned.
         *@see BFS(VID) and GetIncidentEdgesVData(VID)
         */
       vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetCCEdgesVData(VERTEX&) const;

       /**Get a list which contains a list of edges in same Connected Components.
         *If there are n components in this graph then there will be n edge lists in
         *this returned list.
         *@return 2D vector ccedges[i,j] = jth edge of ith CC, edge is VERTEX pair.
         *@see GetCCStats
         */
       vector< vector< pair<VERTEX,VERTEX> > > GetEdgesByCCVDataOnly() const; 

       /**Get all Connected Components from this graph.
         *@return A list of pairs. The dirst element in each pair is the size
         *of component and the second element is the vertex defines this component.
         *@note the list is ordered by the size of component, the first element in
         *the list has largest size.
         *@see CCVID_Compare
         */
       vector< pair<int,VID> > GetCCStats() const;

       /**Get number of Connected Components in this graph.
         *@see GetCCStats
         */
       int GetCCcount() const;

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods*/
    //@{

       /**Output Connected Component defined by VID information to standard output.
         *This method output every VID in this Connected Component.
         *Following format will be output, CC[vids] ={ (v1,v2),... }
         *@param VID this id defines a Connected Component.
         *@see GetCC(VID)
         */
       void DisplayCC(VID) const;

       /**Output edges in connected components.
         *Following format will be output, CC[i] ={ (v1,v2),... }
         *here i is ith connected component in the graph.
         *v1 and v2 are user data associated with a edge.
         *@see GetEdgesByCCVDataOnly
         */
       void DisplayEdgesByCCVDataOnly() const; 

       /**Output Connected Component information in graph.
         *This method will print out number of Connected Component,
         *,and size and start vertex of each Connected Component.
         */
       void DisplayCCStats(int _numCCtoPrint = -1) const; // default print all 

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Utility Stuff
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Utility Stuff*/
    //@{

       ///Modified for VC
       /**This seems VC is not be able to use vector<>::iterator directly without
         *"using namespace std;". how every this causes bigger problem.
         *therefore, one more typedef is used here
         */
       typedef  vector< Vertex > VERTEX_VECTOR;
       typedef  typename VERTEX_VECTOR::iterator VI;                 ///<VI Vertex Iterator
       typedef  typename VERTEX_VECTOR::const_iterator CVI;          ///<CVI Constant Vertex Iterator
       typedef  typename VERTEX_VECTOR::reverse_iterator RVI;        ///<RVI Reverse Vertex Iterator
       typedef  typename VERTEX_VECTOR::const_reverse_iterator CRVI; ///<CRVI Constant Reverse Vertex Iterator

       typedef  vector< WtEdge > WtEdge_VECTOR; 
       typedef  typename WtEdge_VECTOR::iterator EI;                 ///<EI Edge Iterator
       typedef  typename WtEdge_VECTOR::const_iterator CEI;          ///<CEI Constant Edge Iterator
       typedef  typename WtEdge_VECTOR::reverse_iterator REI;        ///<REI Reverse Edge Iterator
       typedef  typename WtEdge_VECTOR::const_reverse_iterator CREI; ///<CREI Constant Reverse Edge Iterator

       /**Compare the size of two given connected components.
         *@return (_cc1.first>_cc2.first)
         *@see GetCCStats
         */
#ifdef __HP_aCC
       static bool CCVID_Compare (const pair<int,VID> _cc1, const pair<int,VID> _cc2); 
#else
       static bool CCVID_Compare (const pair<int,VID>& _cc1, const pair<int,VID>& _cc2); 
#endif

       /**Transform DAG to undirected graph.
         *@note This given WeightedMultiDiGraph should not have both v1->v2 and v2->v1
         *in the graph, otherwise this process will fail and error message will
         *be printed to stand output.
         */
       WeightedGraph<VERTEX,WEIGHT> DagToUndirected(WeightedMultiDiGraph<VERTEX, WEIGHT>&);

     //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected : Adding & Deleting Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Adding & Deleting Edges*/
    //@{

       /**Add a new edge from vid to EI->vertex2id and from EI->vertex2id to vid.
         *Acutelly, this edgae, EI, has been created, this method
         *just adds this edge to vid's edge list.
         *@return 1 if vid1 and/or EI->vertex2id are not in graph.
         *@return 1 if vid1 and vid2 have been connected.
         *0 is every thing is fine.
         *@note enven 2 edges are created, but they are counted as one edge.
         */
        virtual int  AddEdge(VID, EI);

    //@}

   
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:

};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          dfsinfo Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////

/// Auxillary Data Structure for Depth First Search Algorithm
class dfsinfo {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
        ///Default Constructor. Do nothing.
        dfsinfo() {}

        /**Constructor with memory reservation.
          *@param _numVerts How many speaces are going to be reserved.
          *This value should be number of vertex in the graph.
          *@note this method initialize all data member to 0, NULL, or false.
          */
        dfsinfo(int _numVerts) {
            vnode.reserve(_numVerts+1);
            color.reserve(_numVerts+1);
            finish_time.reserve(_numVerts+1);
            
            for( int i=0; i<_numVerts+1; i++) {
                color[i] = 0;     
                vnode[i] = 0;    
                finish_time[i] = 0;    
            }
        }

     //@}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    vector< pair <VID, VID> > backedge_vector;  ///used in cycle detection
    vector<int> finish_time;                    ///<Finish times for all vertex.
    vector<VID> vnode;                          ///<Used as a stack to store vertex visited in seqence
    vector<int> color;                          ///<Colors for each vertex. 0: new, 1: visited, 2: finish
};


//=====================================================================
//=====================================================================
//  METHODS FOR GRAPH RELATED CLASSES
//=====================================================================
//=====================================================================

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template AbstractGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////


//==================================
// AbstractGraph class Methods: Constructors and Destructor
//==================================
template<class VERTEX> 
AbstractGraph<VERTEX>::
AbstractGraph() {
    vertIDs=numVerts=numEdges=0;
    reserveEdgesPerVertex = 0;
}

template<class VERTEX> 
AbstractGraph<VERTEX>::
AbstractGraph(int _reserveEdgesPerVertex) {
    vertIDs=numVerts=numEdges=0;
    reserveEdgesPerVertex = _reserveEdgesPerVertex;
}

template<class VERTEX> 
AbstractGraph<VERTEX>::
~AbstractGraph() {
}

//==================================
// AbstractGraph class Methods: Statistics -- num verts/edges, etc 
//==================================

template<class VERTEX> 
int 
AbstractGraph<VERTEX>:: 
GetVertexCount() const {
    return numVerts;
}

template<class VERTEX> 
int 
AbstractGraph<VERTEX>:: 
GetEdgeCount() const {
    return numEdges;
}

template<class VERTEX>
VID
AbstractGraph<VERTEX>::
GetNextVID() const {
    return vertIDs;
}

//==================================
// AbstractGraph class Methods: Modify Data -- num verts/edges, etc 
//==================================
//void SetStartID(VID);     //To define start number of identifier
//void SetnumVerts(int);
//void SetnumEdges(int);

template<class VERTEX> 
void 
AbstractGraph<VERTEX>:: 
SetStartID(VID _startvid) {
    vertIDs = _startvid;
}

template<class VERTEX> 
void 
AbstractGraph<VERTEX>:: 
SetnumVerts(int _num) {
    numVerts = _num;
}

template<class VERTEX> 
void 
AbstractGraph<VERTEX>:: 
SetnumEdges(int _num) {
    numEdges = _num;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template WeightedMultiDiGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////


//==================================
// WeightedMultiDiGraph class Methods: Constructors and Destructor
//==================================

template<class VERTEX,class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
WeightedMultiDiGraph(){
}

template<class VERTEX,class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
WeightedMultiDiGraph(int _sz) {
    v.reserve(_sz);
}

template<class VERTEX,class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
WeightedMultiDiGraph(int _sz, int _edgelistsz)
: AbstractGraph<VERTEX> (_edgelistsz) 
{
    v.reserve(_sz);
}

template<class VERTEX, class WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
~WeightedMultiDiGraph(){
}


//==================================
// WeightedMultiDiGraph class Methods: Adding & Deleting Vertices
//==================================

template<class VERTEX, class WEIGHT> 
VID 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v) {
    VID vid = this->vertIDs++;
    Vertex newVertex(_v,vid,this->reserveEdgesPerVertex);
    v.push_back(newVertex);
    this->numVerts++;
    return (vid); // return vertex id (not nec. index)
}

template<class VERTEX, class WEIGHT>
VID
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddVertex(vector<VERTEX>& _v) {
    
    if (_v.size()>0) {
        VID vertex_id = AddVertex(_v[0]);
        for (unsigned int i=1;i<_v.size();++i)
            AddVertex(_v[i]);
        return (vertex_id); // return vertex id (not nec. index)
    }
    return INVALID_VID;
    
}

template<class VERTEX, class WEIGHT> 
VID 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v, VID _vid) {
    VID vid = _vid;
    Vertex newVertex(_v,vid,this->reserveEdgesPerVertex);
    v.push_back(newVertex);
    this->numVerts++;
    return (vid); // return vertex id (not nec. index)
}


template<class VERTEX, class WEIGHT> 
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DeleteVertex(VERTEX& _v1) {
    CVI cv1;
    VI v1;
    if ( IsVertex(_v1,&cv1) ) { 
        v1 = const_cast<VI>(cv1);
        DeleteAllEdgesToV(v1->vid);
        v.erase(v1);
        this->numVerts--;
        return 0;
    } else {
        cout << "\nDeleteVertex: vertex not in graph";
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteVertex(VID _v1id) {
    CVI cv1;
    VI v1;
    if ( IsVertex(_v1id,&cv1) ) {
        v1 = const_cast<VI>(cv1);
        DeleteAllEdgesToV(_v1id);
        v.erase(v1);
        this->numVerts--;
        return 0;
    } else {
        cout << "\nDeleteVertex: vertex not in graph";
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
EraseGraph() {
    while ( v.size() != 0 ) 
        v.pop_back();
    this->vertIDs = this->numVerts = this->numEdges = 0;
    return 0;
}


//==================================
// WeightedMultiDiGraph class Methods: Modifying Vertices
//void PutData(VID, VERTEX);
//void SetPredecessors();
//==================================

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
PutData(VID _vid, VERTEX _v){ // lkd: 7-7-99
    CVI cv1;
    VI v1;
    if ( IsVertex(_vid,&cv1) ) {
        v1 = const_cast<VI>(cv1);
        v1->data = _v;
    }
}

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
SetPredecessors() {
    VI v1, v2;
    CVI cv2;
    VID _v2id;
    bool DoneSet = false;

    //check if SetPredecessors() already called
    for(v1 = v.begin(); v1 < v.end(); v1++) {
    if(!v1->predecessors.empty()) {
        DoneSet = true;
        cout<<"\nSetPredecessors() already called."<<endl;
        return;
    }
    }

    for(v1 = v.begin(); v1 < v.end(); v1++) {
        for (EI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            _v2id = ei->vertex2id;
            if ( IsVertex(_v2id, &cv2) ) {
                v2 = const_cast<VI> (cv2);
                WtEdge newEdge( v1->vid, ei->weight );
                v2->predecessors.push_back(newEdge);
            }
        }
    }
}

//==================================
// WeightedMultiDiGraph class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, EI _ei) {
    VI v1/*, v2*/;
    CVI cv1, cv2;
    VID v2id = _ei->vertex2id;
    WEIGHT weight = _ei->weight;
    
    if (IsVertex(_v1id,&cv1) && IsVertex(v2id,&cv2) ) {
        v1 = const_cast<VI> (cv1);
        v1->AddEdge(v2id,weight);
        this->numEdges++;
        return 0;
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}


template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, WEIGHT _weight) {
    CVI cv1;
    VI v1;
    if (IsVertex(_v1id,&cv1) && IsVertex(_v2id) ) {
        v1 = const_cast<VI>(cv1);
        v1->AddEdge(_v2id,_weight);
        this->numEdges++;
        return 0;
    } else {
        cout << endl << "AddEdge: v1 " << _v1id << " and/or v2 " << _v2id << "not in graph" ;
        return 1;
    }
}


template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, pair<WEIGHT,WEIGHT> _wtpair ) {
    CVI cv1,cv2;
    VI v1,v2;
    if (IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        v1->AddEdge(_v2id,_wtpair.first);
        v2->AddEdge(_v1id,_wtpair.second);
        this->numEdges += 2;
        return 0;
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    CVI cv1, cv2;
    VI v1, v2;
    if (IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        v1->AddEdge(v2->vid,_weight);
        this->numEdges++;
        return 0;
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}


template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, double _weight) {
    CVI cv1, cv2;
    VI v1, v2;
    if (IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        v1->AddEdge(v2->vid,_weight);
        this->numEdges++;
        return 0;
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    };
};

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT> _wtpair) {
    CVI cv1, cv2;
    VI v1, v2;
    if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
         v1 = const_cast<VI>(cv1);
         v2 = const_cast<VI>(cv2);
         v1->AddEdge(v2->vid,_wtpair.first);
         v2->AddEdge(v1->vid,_wtpair.second);
         this->numEdges += 2;
         return 0;
      } else {
         cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
         return 1;
     }
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPredecessorEdge(VID _v1id, VID _v0id, WEIGHT _weight) {
    CVI cv1;
    VI v1;
    if (IsVertex(_v1id,&cv1) && IsVertex(_v0id) ) {
         v1 = const_cast<VI>(cv1);
         v1->AddPredecessorEdge(_v0id,_weight);
         return 0;
     } else {
         cout << endl << "AddPredecessorEdge: v1 " << _v1id << " and/or v0 " << _v0id << "not in graph" ;
         return 1;
     }
}


/*
template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector<VID>& _path, WEIGHT _wt) {
    unsigned int i;
    for (i = 0; i < _path.size(); i++){
        if (!IsVertex(_path[i])) return 1;
    }
    for (i = 0; i < _path.size() - 1; i++){
        AddEdge(_path[i],_path[i+1],_wt);
    }
    return 0;
}


template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector<VERTEX>& _path, WEIGHT _wt) {
    if (!IsVertex(_path[0])) AddVertex(_path[0]);
    for (unsigned int i = 0; i < _path.size() - 1; i++){
        if (!IsVertex(_path[i+1])) AddVertex(_path[i+1]);
        AddEdge(_path[i],_path[i+1],_wt);
    }
    return 0;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VID,WEIGHT> >& _path) {
    unsigned int i;
    for (i = 0; i < _path.size(); i++){
        if (!IsVertex(_path[i].first)) return 1;
    }
    for (i = 0; i < _path.size() - 1; i++){
        AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return 0;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VERTEX,WEIGHT> >& _path) {
    if (!IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (unsigned int i = 0; i < _path.size() - 1; i++){
        if (!IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
        AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return 0;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& _path) {
    unsigned int i;
    for (i = 0; i < _path.size(); i++){
        if (!IsVertex(_path[i].first)) return 1;
    }
    for (i = 0; i < _path.size() - 1; i++){
        AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return 0;
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& _path) {
    if (!IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (unsigned int i = 0; i < _path.size() - 1; i++){
        if (!IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
        AddEdge(_path[i].first,_path[i+1].first,_path[i].second);
    }
    return 0;
}
*/

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesToV(VID _v2id) {
    CVI v2;
    if ( IsVertex(_v2id,&v2) ) {
        for (VI vi = v.begin(); vi < v.end(); vi++) {
            this->numEdges -= vi->DeleteXEdges(_v2id,-1);
        }
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VID _v1id) {
    CVI cv1;
    VI v1;
    if ( IsVertex(_v1id,&cv1) ) {
        v1 = const_cast<VI>(cv1);
        this->numEdges -= v1->edgelist.size();
        v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VID _vid) {
    DeleteAllEdgesToV(_vid);
    DeleteAllEdgesFromV(_vid);
}

// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VID _v1id, VID _v2id, int _n) {
    VI v1;
    CVI cv1;
    if ( IsVertex(_v1id,&cv1) ) {
        v1 = const_cast<VI> (cv1);
        this->numEdges -= v1->DeleteXEdges(_v2id,_n);
        return 0;
    } else {
        return 1;
    }
}

// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _weight, int _n) {
    VI v1;
    CVI cv1;
    if ( IsVertex(_v1id,&cv1) ) {
        v1 = const_cast<VI> (cv1);
        this->numEdges -= v1->DeleteXEdges(_v2id,_weight,_n);
        return 0;
    } else {
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesToV(VERTEX& _v2) {
    CVI v2;
    if ( IsVertex(_v2,&v2) ) {
        for (VI vi = v.begin(); vi < v.end(); vi++) {
            this->numEdges -= vi->DeleteXEdges(v2->vid,-1);
        }
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VERTEX& _v1) {
    CVI cv1;
    VI v1;
    if ( IsVertex(_v1,&cv1) ) {
        v1 = const_cast<VI>(cv1); 
        this->numEdges -= v1->edgelist.size();
        v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VERTEX& _v) {
    DeleteAllEdgesToV(_v);
    DeleteAllEdgesFromV(_v);
}


// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n) {
    VI v1,v2;
    CVI cv1, cv2;
    if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        v1= const_cast<VI> (cv1);
        v2= const_cast<VI> (cv2);
        this->numEdges -= v1->DeleteXEdges(v2->vid,_n);
        return 0;
    } else {
        return 1;
    }
}


// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, int _n) {
    VI v1,v2;
    CVI cv1, cv2;
    if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        v1= const_cast<VI> (cv1);
        v2= const_cast<VI> (cv2);
        this->numEdges -= v1->DeleteXEdges(v2->vid,_weight,_n);
        return 0;
    } else {
        return 1;
    }
}


//==================================
// WeightedMultiDiGraph class Methods: Finding Vertices & Edges
//==================================

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id) const {
    CVI v1;
    return ( IsVertex(_v1id,&v1) );
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id, const Vertex**  _v1ptr) const {
    
    CVI v1 = my_find_VID_eq(_v1id);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;        
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1) const {
    CVI v1;
    return ( IsVertex(_v1,&v1) );
}


template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1, const Vertex**  _v1ptr) const {
    
    CVI v1 = my_find_VDATA_eq(_v1);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id) const {
    CVI v1;
    CEI e12;
    return ( IsEdge(_v1id,_v2id,&v1,&e12) );
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, WEIGHT _weight) const {
    CVI v1;
    CEI e12;
    return ( IsEdge(_v1id,_v2id,_weight,&v1,&e12) );
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, const Vertex** _v1ptr, const WtEdge** _e12ptr) const {
    CVI v1;
    CEI e12;
    if ( IsVertex(_v1id,&v1) ) {
        if (v1->IsEdge(_v2id,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, WEIGHT _weight, const Vertex** _v1ptr, const WtEdge** _e12ptr)  const {
    CVI v1;
    CEI e12;
    if ( IsVertex(_v1id,&v1) ) {
        if (v1->IsEdge(_v2id,_weight,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2) const {
    CVI v1;
    CEI e12;
    return ( IsEdge(_v1,_v2,&v1,&e12) );
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) const {
    CVI v1;
    CEI e12;
    return ( IsEdge(_v1,_v2,_weight,&v1,&e12) );
}


template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, const Vertex** _v1ptr, const WtEdge** _e12ptr) const {
    CVI v1,v2;
    CEI e12;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
        if (v1->IsEdge(v2->vid,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, const Vertex** _v1ptr, const WtEdge** _e12ptr) const {
    CVI v1,v2;
    CEI e12;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
        if (v1->IsEdge(v2->vid,_weight,&e12)) {
            *_v1ptr = v1;
            *_e12ptr = e12;
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

//==================================
// WeightedMultiDiGraph class Methods: Getting Data & Statistics
//==================================

template<class VERTEX, class WEIGHT>
vector<VID> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesVID() const {
    vector<VID> verts;
    verts.reserve( v.size() );
    for (CVI vi = v.begin(); vi < v.end(); vi++) {
        verts.push_back(vi->vid);
    }
    sort( verts.begin(),verts.end() );
    return verts;
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesVID(VID _vid, int _n) const {
    vector<VID> verts;
    verts.reserve( _n );
    CVI v1, v2;
    unsigned int i;
    if ( IsVertex(_vid,&v1) ) {
        for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
            verts.push_back(v2->vid);
        }
    } else {
        cout << "\nIn GetVerticesVID(VID,int): no vertex VID=" << _vid << " in graph\n";
    }
    sort( verts.begin(),verts.end() );
    return verts;
}

template<class VERTEX, class WEIGHT>
vector<VERTEX> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesData() const {
    vector<VERTEX> verts;
    verts.reserve( v.size() );
    for (CVI vi = v.begin(); vi < v.end(); vi++) {
        verts.push_back(vi->data);
    }
    return verts;
}

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVerticesData(VID _vid, int _n) const {
    vector<VERTEX> verts;
    verts.reserve( _n );
    CVI v1, v2;
    unsigned int i;
    if ( IsVertex(_vid,&v1) ) {
        for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
            verts.push_back(v2->data);
        }
    } else {
        cout << "\nIn GetVerticesData(VID,int): no vertex VID=" << _vid << " in graph\n";
    }
    return verts;
}


template<class VERTEX, class WEIGHT>
vector< pair< pair<VID,VID>, WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetEdges() const {
    vector< pair< pair<VID,VID>, WEIGHT> > edges;
    
    edges.reserve(this->numEdges);
    for (CVI vi = v.begin(); vi != v.end(); vi++) {
        for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
            pair<VID,VID> newedge(vi->vid, ei->vertex2id);
            pair<pair<VID,VID>,WEIGHT> newedgewt(newedge, ei->weight);
            edges.push_back( newedgewt );
        }
    }
    return edges;
}

template<class VERTEX, class WEIGHT>
vector< pair< pair<VERTEX,VERTEX>, WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetEdgesVData() const {
    vector< pair< pair<VERTEX,VERTEX>, WEIGHT> > edges;
    
    edges.reserve(this->numEdges);
    for (CVI vi = v.begin(); vi != v.end(); vi++) {
        for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
            VERTEX v2data = GetData(ei->vertex2id);
            pair<VERTEX,VERTEX> newedge(vi->data, v2data);
            pair<pair<VERTEX,VERTEX>,WEIGHT> newedgewt(newedge, ei->weight);
            edges.push_back( newedgewt );
        }
    }
    return edges;
}

template<class VERTEX, class WEIGHT>
VERTEX
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetData(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return v1->data;
    } else {
        return VERTEX::InvalidData(); 
    }
}

template<class VERTEX, class WEIGHT>
VERTEX*
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetReferenceofData(VID _v1id) {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return const_cast<VERTEX*> (&v1->data);
    } else {
//  VERTEX vv = VERTEX::InvalidData();
        return NULL; 
    }
}

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetData(VID _v1id, VID _v2id) const {
    CVI v1, v2;
    vector<VERTEX> vset;
    
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) ) {
        for (VID i = _v1id; i <= _v2id; i++) { 
            vset.push_back(v1->data);
            v1++;
        }
        return vset;
    } else {
        return vset; //in this case return an empty vector
    }
}


template<class VERTEX, class WEIGHT>
VID
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVID(VERTEX& _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
        return v1->vid;
    } else {
        return INVALID_VID;
    }
}

template<class VERTEX, class WEIGHT>
int
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetVertexOutDegree(VID _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
        return v1->edgelist.size();
    } else {
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSuccessors(VID _v1id) const {
    vector<VID> succ;
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        succ.reserve( v1->edgelist.size() );
        for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            succ.push_back(ei->vertex2id);
        }
    } else {
        cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return succ;
}

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSuccessorsDATA(VID _v1id) const {
     vector<VERTEX> succ;
     CVI v1,v2;
     if ( IsVertex(_v1id,&v1) ) {
         succ.reserve( v1->edgelist.size() );
         for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
       if ( IsVertex(ei->vertex2id,&v2) )
         succ.push_back(v2->data);
         }
     } else {
         cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
     }
     return succ;
}
template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSuccessors(VERTEX& _v1) const {
  return GetSuccessors( GetVID(_v1) );
}


template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSuccessorsDATA(VERTEX& _v1) const {
  return GetSuccessorsDATA( GetVID(_v1) );
}


template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetPredecessors(VID _v1id) const {
    vector<VID> pred;
    CVI cv1,v1;
    if ( IsVertex(_v1id,&cv1) ) {
        v1=const_cast<VI> (cv1);
        pred.reserve( v1->predecessors.size() );
        for (CEI ei = v1->predecessors.begin(); ei != v1->predecessors.end(); ei++) {
            pred.push_back(ei->vertex2id);
         }
     } else {
         cout << "\nGetPredecessors: vertex "<< _v1id << " not in graph";
     }
     return pred;
}
template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetPredecessorsDATA(VID _v1id) const {
     vector<VERTEX> pred;
     CVI cv1,v1,v2;
     if ( IsVertex(_v1id,&cv1) ) {
        v1=const_cast<VI> (cv1);
         pred.reserve( v1->predecessors.size() );
         for (CEI ei = v1->predecessors.begin(); ei != v1->predecessors.end(); ei++) {
        if ( IsVertex(ei->vertex2id,&v2) )  
          pred.push_back(v2->data);
         }
     } else {
         cout << "\nGetPredecessors: vertex "<< _v1id << " not in graph";
     }
     return pred;
}
template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetPredecessors(VERTEX& _v1) const {
  return GetPredecessors( GetVID(_v1) );
}

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetPredecessorsDATA(VERTEX& _v1) const {
  return GetPredecessorsDATA( GetVID(_v1) );
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSources() const {
    vector<VID> sourcevids;
    CVI cv1;
    VI v1;
    for(cv1=v.begin(); cv1!=v.end(); cv1++) {
        v1 = const_cast<VI> (cv1);
        if(v1->predecessors.empty()) sourcevids.push_back(v1->vid);
    }
    return sourcevids;
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetSinks() const {
    vector<VID> sinkvids;
    CVI cv1;
    VI v1;
    for(cv1=v.begin(); cv1!=v.end(); cv1++) {
//        v1 = const_cast<VI> (cv1);
    if(cv1->edgelist.empty()) sinkvids.push_back(cv1->vid);
    }
    return sinkvids;
}

template<class VERTEX, class WEIGHT>
WEIGHT 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
GetEdgeWeight(VID _v1id, VID _v2id) const {
    CVI v1;
    CEI e12;
    if (IsEdge(_v1id,_v2id,&v1,&e12)) {
        return  e12->weight;
    } else {
        return WEIGHT::InvalidWeight();
    }
}

template<class VERTEX, class WEIGHT>
WEIGHT 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
GetEdgeWeight(VERTEX& _v1, VERTEX& _v2) const {
    
    return GetEdgeWeight( GetVID(_v1), GetVID(_v2) );
}

//==================================
// WeightedMultiDiGraph class Methods: Basic Graph Algorithms
//==================================

//*************************************************************** 
//  BREADTH-FIRST-SEARCH ALGORITHMS
//*************************************************************** 

template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFS (VERTEX& _startV) const {
    CVI cv1;
    if ( IsVertex(_startV,&cv1) ) {
        return BFS(cv1->vid);
    } else {
        cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
        WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
        return bfstree; 
    }
}


template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT> 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFS (VID _startVid) const {
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
    list<VID> q; 
    CVI cv1,cv2;
    VI v1,v2;
    VID v1id, v2id; 
    
    if ( IsVertex(_startVid,&cv1) ) {
        q.push_back(_startVid);
        v1 = const_cast<VI>(cv1);
        bfstree.AddVertex(v1->data,_startVid); 
    } else {
        cout << "\nIn GraphBFS: root vid=" << _startVid << " not in graph";
        return bfstree; 
    }
    
    while ( !q.empty() ) {
        v1id = q.front();
        if ( IsVertex(v1id,&cv1) ) {
            for (CEI e = cv1->edgelist.begin(); e < cv1->edgelist.end(); e++) {
                v2id = e->vertex2id;
                if ( !bfstree.IsVertex(v2id) && IsVertex(v2id,&cv2) ) { 
                    q.push_back(v2id);
                    v2 = const_cast<VI>(cv2);
                    bfstree.AddVertex(v2->data,v2id); 
                    if ( bfstree.AddEdge(v1id,v2id,e->weight) != 0) {
                        cout << "\nIn GraphBFS: OOPS! edge not added right...";
                    }
                }
            }
        } else {   
            cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
        }
        q.pop_front();
    }
    bfstree.vertIDs = this->vertIDs; // set the same vert ID as in graph
    return bfstree;
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFSVID (VERTEX& _startV) const {
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_startV); 
    return bfstree.GetVerticesVID();
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
BFSVID (VID _startVID) const {
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = BFS(_startVID); 
    return bfstree.GetVerticesVID();
}


//*************************************************************** 
//  DEPTH-FIRST-SEARCH ALGORITHMS
//*************************************************************** 
template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DFS (VERTEX& _startV) const {
    CVI cv1; VI v1;
    if ( IsVertex(_startV,&cv1) ) {
      v1 = const_cast<VI> (cv1);
        return DFS(v1->vid);
    } else {
        cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
        WeightedMultiDiGraph<VERTEX,WEIGHT> dfstree;
        return dfstree; 
    }
}

template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DFS (VID & vid) const {
//To find one dfs tree w/ start vertex vid
 
  dfsinfo dfs(this->GetVertexCount());
  return true_DFS(vid, dfs);
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DFSVID (VERTEX& _startV) const {
    WeightedMultiDiGraph<VERTEX,WEIGHT> dfstree = DFS(_startV); 
    return dfstree.GetVerticesVID();
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DFSVID (VID& _startVID) const {
    WeightedMultiDiGraph<VERTEX,WEIGHT> dfstree = DFS(_startVID); 
    return dfstree.GetVerticesVID();
}

template<class VERTEX, class WEIGHT>
vector<WeightedMultiDiGraph<VERTEX,WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DFS () const {
//To find all dfs trees
    vector< WeightedMultiDiGraph<VERTEX,WEIGHT> > dfstree_vector;
    CVI cv1; VI  v1;
    VID vid;    
    dfsinfo dfs(this->GetVertexCount());

    for (cv1 = v.begin(); cv1 < v.end(); cv1++) {
      v1 = const_cast<VI> (cv1);
        vid = v1->vid;
            if( dfs.color[vid] == 0 ) 
            dfstree_vector.push_back(true_DFS(vid, dfs));
    }
    return dfstree_vector;
}


template<class VERTEX, class WEIGHT>
bool
WeightedMultiDiGraph<VERTEX,WEIGHT>::
IsCycle () const {
    dfsinfo dfs(this->GetVertexCount());
    aux_DFS(dfs);
    if( dfs.backedge_vector.empty() ) return false;
    else return true;
}

template<class VERTEX, class WEIGHT>
vector< pair<VID,VID> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
GetBackedge() const {
    dfsinfo dfs(this->GetVertexCount());
    aux_DFS(dfs);
    return dfs.backedge_vector;
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
TopologicalSort () const {
    unsigned int i,n;
    vector<VID> tps;
    n=this->GetVertexCount();
    vector<pair<VID,int> > tmp;
    tmp.reserve(n);

    dfsinfo dfs(n);
    aux_DFS(dfs);
    
    for(i=1;i<=n;i++) {
        pair<VID,int> newpair(i,dfs.finish_time[i]);
        tmp.push_back(newpair);
    }   
    
    stable_sort(tmp.begin(),tmp.end(),ptr_fun(FinishLate));
    
    for(i=0; i<n;i++) {
        tps.push_back(tmp[i].first);
/*
#if DEBUG
    cout<<"\nTopological Sort results: "<<endl;
        cout<<tmp[i].first<<" ";
#endif
*/
    }
    return tps;            
}

template<class VERTEX, class WEIGHT>
bool
WeightedMultiDiGraph<VERTEX,WEIGHT>::
#ifdef __HP_aCC
FinishLate(const pair<VID,int> _x, const pair<VID,int> _y) {
#else 
FinishLate(const pair<VID,int>& _x, const pair<VID,int>& _y) {
#endif
    return _x.second > _y.second;
} 

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>::
aux_DFS (dfsinfo& dfs) const {

//driver, to find all backedges, and topological order of vertices
 
  CVI cv1;
  VI  v1;
  VID vid;  

  for (cv1 = v.begin(); cv1 < v.end(); cv1++) {
    v1 = const_cast<VI> (cv1);
        vid = v1->vid;
        if( dfs.color[vid] == 0 ) 
        true_DFS(vid, dfs);
  }

}

template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
true_DFS (VID & vid, dfsinfo & dfs) const {
    //core DFS algorithm, start w/ vertex vid
    //return a dfstree started at vid
    //also return dfsinfo
    
    CVI cv1,cv2;
    VI  v1;
    VID v1id=0,v2id=0;  //v1id=parent(vid), v2id=adj(vid)
    int k=1;
    static int ftime=0; //static finish time,  the finish time for different 
    //trees is in consecutive order
    WeightedMultiDiGraph<VERTEX,WEIGHT> dfstree;
    
    dfs.color[vid] = 1;
    dfs.vnode[k] = vid;  
    if ( !IsVertex(vid, &cv1)) 
        cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
    v1 = const_cast<VI>(cv1);
    dfstree.AddVertex(v1->data,vid);
    while( k > 0 ) {
        vid = dfs.vnode[k];
        if ( !IsVertex(vid, &cv1)) 
            cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
        v1 = const_cast<VI>(cv1);
        CEI e = v1->edgelist.begin(); 
        while ( e < v1->edgelist.end() ) {
            v2id = e->vertex2id;
            if( !dfs.color[v2id]) {
                v1id = vid; //parent
                vid = v2id; //child
                dfs.color[vid] = 1;
                dfs.vnode[++k] = vid;
                if ( IsVertex(vid, &cv2) ) {
                    v1 = const_cast<VI>(cv2); //set current v as child
                    dfstree.AddVertex(v1->data,vid);
                    dfstree.AddEdge(v1id,vid,e->weight);
                    e = v1->edgelist.begin();
                } else cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
            }  
            else if( dfs.color[v2id] == 1) {    //gray
                pair<VID,VID> backedge(v2id,vid);
                dfs.backedge_vector.push_back(backedge); //record back edge
                e++;
                cout<<"backedge:"<<v2id<<" " << vid<<endl;
            }
            else e++;
        }
        dfs.finish_time[dfs.vnode[k]]=++ftime;
        dfs.color[dfs.vnode[k]] = 2; //black
        k--; //back to parent
    }
    //dfstree.DisplayGraph();
    return dfstree;
}

//==============================
//  FindPathBFS (for any graph)
//  -- returns BFS path between 2 specified vertices 
//==============================
template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathBFS (VERTEX& _startV, VERTEX& _endV) const {
    
    CVI cv1,cv2;
    if ( IsVertex(_startV,&cv1) && IsVertex(_endV,&cv2) ){
        return FindPathBFS(cv1->vid,cv2->vid);
    } else {
        cout << "\nIn FindPathBFS: start or goal vertex (";
        cout << _startV << ", " << _endV << ") not in graph";
        vector< pair<VERTEX,WEIGHT> > path;
        return path; 
    }
}


template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> > 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathBFS (VID _startVid, VID _endVid) const {

  WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
  vector< pair<VERTEX,WEIGHT> > path;
  list<VID> q;
  CVI cv1,cv2;
  VI v1,v2;
  VID v1id, v2id;


  if ( IsVertex(_startVid,&cv1) ) {
     path.reserve( v.size() );
     q.push_back(_startVid);
     v1 = const_cast<VI>(cv1);
     bfstree.AddVertex(v1->data,_startVid); 
  } else {
     cout << "\nIn FindPathBFS: start vertex (" << _startVid << ") not in graph";
     return path; 
  }

  while ( !q.empty() && !bfstree.IsVertex(_endVid) ) {
     v1id = q.front();
     if ( IsVertex(v1id,&cv1) ) {
       for (CEI e = cv1->edgelist.begin(); e < cv1->edgelist.end(); e++) {
         v2id = e->vertex2id;
         if ( !bfstree.IsVertex(v2id) && IsVertex(v2id,&cv2) ) {
            q.push_back(v2id);
            v2 = const_cast<VI>(cv2);
            bfstree.AddVertex(v2->data,v2id);
            if ( bfstree.AddEdge(v2id,v1id,e->weight) != 0) {
                cout << "\nIn FindPathBFS: OOPS! edge not added right...";
            }
         }
       }
     } else {
       cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
     }
     q.pop_front();
  }

  if ( bfstree.IsVertex(_endVid,&cv1) && bfstree.IsVertex(_startVid,&cv2) ) {
     path.insert( path.begin(), pair<VERTEX,WEIGHT>(cv1->data,WEIGHT::InvalidWeight() ) );
     while ( !(path.begin()->first ==  cv2->data) ) {
        CEI e = cv1->edgelist.begin();
        v1id = cv1->vid;
        v2id = e->vertex2id;
        if ( bfstree.IsVertex(v2id,&cv1) ) {
           path.insert( path.begin(), pair<VERTEX,WEIGHT>(cv1->data,e->weight) );
           bfstree.DeleteEdge(v1id,v2id);
        } else {
           cout << "In FindPathBFS: hmm....\n";
        }
     }
  }
  return path;
}


template<class VERTEX, class WEIGHT>
vector< VID > 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindVIDPathBFS (VID _startVid, VID _endVid) const {
  WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree;
  vector< VID > path;
  list<VID> q;
  CVI cv1,cv2;
  VI v1,v2;
  VID v1id, v2id;


  if ( IsVertex(_startVid,&cv1) ) {
     path.reserve( v.size() );
     q.push_back(_startVid);
     v1 = const_cast<VI>(cv1);
     bfstree.AddVertex(v1->data,_startVid); 
  } else {
     cout << "\nIn FindPathBFS: start vertex (" << _startVid << ") not in graph";
     return path; 
  }

  while ( !q.empty() && !bfstree.IsVertex(_endVid) ) {
     v1id = q.front();
     if ( IsVertex(v1id,&cv1) ) {
       for (CEI e = cv1->edgelist.begin(); e < cv1->edgelist.end(); e++) {
         v2id = e->vertex2id;
         if ( !bfstree.IsVertex(v2id) && IsVertex(v2id,&cv2) ) {
            q.push_back(v2id);
            v2 = const_cast<VI>(cv2);
            bfstree.AddVertex(v2->data,v2id);
            if ( bfstree.AddEdge(v2id,v1id,e->weight) != 0) {
                cout << "\nIn FindPathBFS: OOPS! edge not added right...";
            }
         }
       }
     } else {
       cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
     }
     q.pop_front();
  }

  if ( bfstree.IsVertex(_endVid,&cv1) && bfstree.IsVertex(_startVid,&cv2) ) {
     path.insert( path.begin(),cv1->vid);
     while ( !(*path.begin() ==  cv2->vid) ) {
        CEI e = cv1->edgelist.begin();
        v1id = cv1->vid;
        v2id = e->vertex2id;
        if ( bfstree.IsVertex(v2id,&cv1) ) {
           path.insert( path.begin(),cv1->vid);
           bfstree.DeleteEdge(v1id,v2id);
        } else {
          // cout << "In FindPathBFS: hmm....\n";
           break;
    }
     }
  }
  return path;
}

//*************************************************************** 
//  DIJKSTRA'S ALGORITHM
//*************************************************************** 
template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
#ifdef __HP_aCC
dkinfo_Compare ( const dkinfo _d1, const dkinfo _d2) {
#else 
dkinfo_Compare ( const dkinfo& _d1, const dkinfo& _d2) {
#endif
    return ( _d1.dist > _d2.dist );
}

///////////////////////////////////////////////////////////////////////
// Dijkstra's Algorithm (follows CLR)
// *NOTE* returns a sssp tree where edge weights are distances from source
//        i.e., the edge weights are cumulative path lengths
///////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DijkstraSSSP(VID _startVid) const {
    
    CVI cv1;
    VI v1;
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> sssptree;
    
    // initialize all distances to be big... and predecessors = NULL
    vector<dkinfo> pq;
    double  maxdist = v.size() * WEIGHT::MaxWeight();
    for ( cv1 = v.begin(); cv1 != v.end(); cv1++) {
        v1 = const_cast<VI>(cv1);
        if (v1->vid == _startVid) {
            pq.push_back( dkinfo(v1->vid,INVALID_VID,0) );
        } else {
            pq.push_back( dkinfo(v1->vid,INVALID_VID,maxdist) );
        }
    }
    
    ///Modified for VC
#if defined(_WIN32)
    typedef bool (*Compare_Fun_Ptr)(const dkinfo& _d1, const dkinfo& _d2);
    sort( pq.begin(), pq.end(),  ptr_fun( (Compare_Fun_Ptr)dkinfo_Compare) );
#else
    sort( pq.begin(), pq.end(),  ptr_fun(dkinfo_Compare) );
#endif
    
    // loop through and determine shortest paths 
    while ( pq.size() != 0  && (pq.back().dist < maxdist) ) {
        bool relax = false;
        dkinfo u = pq.back();
        if ( sssptree.GetVertexCount() == 0 ) {
            VERTEX tmp = GetData(u.vid);
            sssptree.AddVertex( tmp );
        } else {
            VERTEX tmp = GetData(u.vid);
            sssptree.AddVertex( tmp );
            VERTEX tmp1 = GetData(u.predvid);
            sssptree.AddEdge( tmp1, tmp, u.dist);
        } 
        pq.pop_back();
        
        // check all u's successors 
        vector<VID> adj = GetSuccessors(u.vid);
        for (unsigned int i = 0; i < adj.size(); i++) {
            for (int j=0; j < pq.size(); j++) {
                if (adj[i] == pq[j].vid) {
                    double wt = GetEdgeWeight(u.vid,pq[j].vid).Weight();
                    // relax 
                    if ( pq[j].dist > u.dist + wt ) { 
                        relax = true;
                        pq[j].dist = u.dist + wt;
                        pq[j].predvid = u.vid;
                    }
                }
            } // endfor
        } // endfor
        
        ///Modified for VC
#if defined(_WIN32)
        typedef bool (*Compare_Fun_Ptr)(const dkinfo& _d1, const dkinfo& _d2);
        if (relax) sort( pq.begin(), pq.end(),  ptr_fun( (Compare_Fun_Ptr)dkinfo_Compare) );
#else
        if (relax) sort( pq.begin(), pq.end(),  ptr_fun(dkinfo_Compare) );
#endif
    } // endwhile
    return sssptree;
} 

///////////////////////////////////////////////////////////////////////
// FindPathDijkstra
// *NOTE* returns shortest path where edge weights are original edge weights
//        i.e., the edge weights are *not* the cumulative path weight
///////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> > 
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathDijkstra (VID _v1id, VID _v2id) const {
    
    // first, get Dijkstra's SSSP tree
    WeightedMultiDiGraph<VERTEX,WEIGHT> sssptree;
    sssptree =  DijkstraSSSP(_v1id); 
    
    // now, get bfspath in sssp tree (there's only one path in tree!)
    vector< pair<VERTEX,WEIGHT> > dpath;
    VERTEX tmp1 = GetData(_v1id);
    VERTEX tmp2 = GetData(_v2id);
    dpath = sssptree.FindPathBFS(tmp1, tmp2);
    
    // now, get "real" edge weights (not the distances in sssptree)
    for (unsigned int i=1; i < dpath.size(); i++) {
        WEIGHT tmp = GetEdgeWeight( dpath[i-1].first, dpath[i].first ); 
        dpath[i-1].second = tmp;
    }
    
    return dpath;
}

///////////////////////////////////////////////////////////////////////
// FindPathDijkstra
// *NOTE* returns shortest path where edge weights are original edge weights
///////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
vector< pair<VERTEX,WEIGHT> >
WeightedMultiDiGraph<VERTEX,WEIGHT>::
FindPathDijkstra (VERTEX& _startV, VERTEX& _endV) const {
    
    CVI cv1,cv2;
    if ( IsVertex(_startV,&cv1) && IsVertex(_endV,&cv2) ){
        return FindPathDijkstra(cv1->vid,cv2->vid);
    } else {
        cout << "\nIn FindPathDijkstra: start or goal vertex (";
        cout << _startV << ", " << _endV << ") not in graph";
        vector< pair<VERTEX,WEIGHT> > path;
        return path;
    }
}


//==================================
// WeightedMultiDiGraph class Methods: Display, Input, Output 
//==================================


template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DisplayGraph() const {
      CVI vi;
      unsigned int i;
      cout<<endl;
      for (vi = v.begin(), i=0; vi < v.end(); vi++, i++) {
          cout << setw(3) << i << ": ";
          vi->DisplayEdgelist();
          cout<<endl;
      }
}

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DisplayVertices() const {
    for (CVI vi = v.begin(); vi != v.end(); vi++) {
        DisplayVertex(vi->vid); 
    } 
}

template<class VERTEX, class WEIGHT>
void
WeightedMultiDiGraph<VERTEX,WEIGHT>::
DisplayVertex(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        cout << "vertex: id =" << setw(3) << _v1id; 
        cout << ", data = [" << v1->data;
        cout << "]" << endl;
    } else {
        cout << "vertex with id=" << _v1id << " not in graph.";
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
DisplayVertexAndEdgelist(VID _v1id) const{
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        cout << "vertex: ";
        v1->DisplayEdgelist();
    } else {
        cout << "vertex with id=" << _v1id << " not in graph.";
    }
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
WriteGraph(const char* _fname) const {
    
    ofstream  myofstream(_fname);
    if (!myofstream) {
        cout << "\nInWriteGraph: can't open outfile: " << _fname ; 
    }
    WriteGraph(myofstream);
    myofstream.close();
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
WriteGraph(ostream& _myostream) const {

      _myostream << endl << "#####GRAPHSTART#####";
      //_myostream << endl << "GRAPHSTART";
      _myostream << endl << this->numVerts << " " << this->numEdges << " " << this->vertIDs; 

      //format: VID VERTEX #edges VID WEIGHT VID WEIGHT ... 
      for (CVI vi = v.begin(); vi != v.end(); vi++) {
          _myostream << endl;
          vi->WriteEdgelist(_myostream);
      } 

      _myostream << endl << "#####GRAPHSTOP#####";
      //_myostream << endl << "GRAPHSTOP";
      _myostream << endl; 
}


template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
ReadGraph(const char*  _fname) {
    
    ifstream  myifstream(_fname);
    if (!myifstream) {
        cout << "\nIn ReadGraph: can't open infile: " << _fname ;
        return;
    }
    ReadGraph(myifstream);
    myifstream.close();
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
ReadGraph(istream& _myistream) {
    VID v1id, v2id, maxVID;
    CVI  cv1;
    VI  v1;
    VERTEX data;
    WEIGHT weight;
    int nVerts, nEdges, nedges;
    char tagstring[100];
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTART") ) {
        cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
        return;
    }
    
    if (this->numVerts != 0) {
        EraseGraph(); // empty graph before filling it in
    }
    
    _myistream >> nVerts >> nEdges >> maxVID;
    
    for ( int i = 0; i < nVerts; i++){
        _myistream >> v1id >> data;             // read and add vertex 
        AddVertex(data,v1id);
        if ( !IsVertex(v1id,&cv1) ) {
            cout << "\nIn ReadGraph: didn't add v1...";
        }
        
        _myistream >> nedges;               // read and add its edges
        v1 = const_cast<VI>(cv1);
        for (int j = 0; j < nedges; j++){
            _myistream >> v2id >> weight; 
            v1->AddEdge(v2id,weight);
         }
      }
      
      this->numVerts = nVerts;
      this->numEdges = nEdges;
      this->vertIDs = maxVID; // set the maximum VID used so far...
                        // should sort verts & find biggest used...

      _myistream >> tagstring;
      if ( !strstr(tagstring,"GRAPHSTOP") ) {
         cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
         return;
      }
}
template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
ReadGraphwithAutoVID(const char*  _fname) {

      ifstream  myifstream(_fname);
      if (!myifstream) {
         cout << "\nIn ReadGraph: can't open infile: " << _fname ;
         return;
      }
      ReadGraphwithAutoVID(myifstream);
      myifstream.close();
}

template<class VERTEX, class WEIGHT>
void 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
ReadGraphwithAutoVID(istream& _myistream) {
      VID v1id, v2id, maxVID;
      CVI  cv1;
      VI  v1;
      VERTEX data;
      WEIGHT weight;
      int nVerts, nEdges, nedges;
      char tagstring[100];

      _myistream >> tagstring;
      if ( !strstr(tagstring,"GRAPHSTART") ) {
         cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
         return;
      }

      if (this->numVerts != 0) {
         EraseGraph(); // empty graph before filling it in
      }

      _myistream >> nVerts >> nEdges >> maxVID;

      for (unsigned int i = 0; i < nVerts; i++){
         _myistream >> data;             // read and add vertex 
         AddVertex(data);
    v1id = i;       //start vid from 0
         if ( !IsVertex(v1id,&cv1) ) {
            cout << "\nIn ReadGraph: didn't add v1...";
         }

         _myistream >> nedges;               // read and add its edges
         v1 = const_cast<VI>(cv1);
         for (int j = 0; j < nedges; j++){
            _myistream >> v2id >> weight; 
            v1->AddEdge(v2id,weight);
         }
      }
      
      this->numVerts = nVerts;
      this->numEdges = nEdges;
      this->vertIDs = maxVID; // set the maximum VID used so far...
                        // should sort verts & find biggest used...

      _myistream >> tagstring;
      if ( !strstr(tagstring,"GRAPHSTOP") ) {
         cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
         return;
      }
}
  //==================================
  // WeightedMultiDiGraph class Predicates, Comparisons & Operations
  //==================================

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>*
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
my_find_VID_eq(const VID _vid) const {
    CVI vi, startvi; 
    
    // find the spot to start looking, hopefully at v[_vid]
    if ( v.size() > _vid ) {
        startvi = v.begin() + _vid;
    } else {
        startvi = v.end() - 1; 
    }
    
    // look back from v[_vid]
    vi = startvi;
    while ( vi >= v.begin()  ) {
        if ( vi->vid == _vid) {
            return const_cast<WtVertexType<VERTEX,WEIGHT>*>( vi );
        } else {
            vi--;
        }
    }
    
    // look forward from v[_vid]
    vi = startvi;
    while ( vi < v.end()  ) {
        if ( vi->vid == _vid) {
            return const_cast<WtVertexType<VERTEX,WEIGHT>*>( vi );
        } else {
            vi++;
        }
    }
    
    // if didn't find it return v.end() like STL find
    return const_cast<WtVertexType<VERTEX,WEIGHT>*>( v.end() );
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>*
WeightedMultiDiGraph<VERTEX,WEIGHT>::
my_find_VDATA_eq(const VERTEX& _v) const {
    CVI cvi = v.begin();
    VI vi;
    bool found = false;
    vi = const_cast<VI> (cvi);
    while (vi != v.end() && !found) {
        if ( vi->data == _v) {
            found = true;
        } else {
            vi++;
        }
    }
    return const_cast<WtVertexType<VERTEX,WEIGHT>*>(vi);
}

template<class VERTEX, class WEIGHT>
bool 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
#ifdef __HP_aCC
VID_Compare (const Vertex _v1, const Vertex _v2){
#else 
VID_Compare (const Vertex& _v1, const Vertex& _v2){
#endif
    return (_v1.vid < _v2.vid ) ; 
}


/*-------------- don't work with sgi/CC, work with sun/g++ -----------------
template<class VERTEX, class WEIGHT>
class 
WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
VID_eq : public unary_function< WeightedMultiDiGraph<VERTEX,WEIGHT>::Vertex,bool> {
public:
explicit VID_eq(const VID vid) : testid (vid) {}
bool operator() (Vertex v) {
return v.vid == testid;
}
protected:
private:
VID testid;
}

  template<class VERTEX, class WEIGHT>
  class
  WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
  VDATA_eq : public unary_function< WeightedMultiDiGraph<VERTEX,WEIGHT>::Vertex, bool> {
  public:
  explicit VDATA_eq(const VERTEX vt) : testdata (vt) {}
  bool operator() (Vertex v) {
  return v.data == testdata;
  }
  protected:
  private:
  VERTEX testdata;
  }
  
    template<class VERTEX, class WEIGHT>
    class 
    WeightedMultiDiGraph<VERTEX,WEIGHT>:: 
    VID_Compare {
    public:
    int operator() (const Vertex & _v1, const Vertex & _v2) const 
    { return (_v1.GetVID() < _v2.GetVID() ); }
    protected:
    private:
    }
    
--------------------------------------------------------------------------*/

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template WeightedGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//==================================
// WeightedGraph class Methods: Constructors and Destructor
//==================================

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
WeightedGraph(){
}

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
WeightedGraph(int _sz)
: WeightedMultiDiGraph<VERTEX,WEIGHT>(_sz) 
{
}

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
WeightedGraph(int _sz, int _edgelistsz)
: WeightedMultiDiGraph<VERTEX,WEIGHT>(_sz,_edgelistsz) 
{
}

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>::
~WeightedGraph(){
}
//==================================
// WeightedGraph class Methods: Adding & Deleting Vertices
//==================================

//==================================
// WeightedGraph class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
int 
WeightedGraph<VERTEX,WEIGHT>:: 
AddEdge(VID _v1id, VID _v2id, WEIGHT _weight) {
    CVI cv1,cv2;
    VI v1,v2;
    if (_v1id != _v2id && IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) ) {
        if ( !this->IsEdge(_v1id,_v2id) ) {
            v1 = const_cast<VI>(cv1);
            v2 = const_cast<VI>(cv2);
            v1->AddEdge(_v2id,_weight);
            v2->AddEdge(_v1id,_weight);
            this->numEdges++;
            return 0;
        } else {
#ifndef QUIETGRAPH
            cout << "\nIn AddEdge: edge already in graph, not added";
#endif
            return 1;
        }
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, pair<WEIGHT,WEIGHT> _wtpair ) {
    CVI cv1,cv2;
    VI v1,v2;
    if (_v1id != _v2id && IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) ) {
        if ( !this->IsEdge(_v1id,_v2id) ) {
            v1 = const_cast<VI>(cv1);
            v2 = const_cast<VI>(cv2);
            v1->AddEdge(_v2id,_wtpair.first);
            v2->AddEdge(_v1id,_wtpair.second);
            this->numEdges++;
            return 0;
        } else {
#ifndef QUIETGRAPH
            cout << "\nIn AddEdge: edge already in graph, not added";
#endif
            return 1;
        }
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}


template<class VERTEX, class WEIGHT>
int 
WeightedGraph<VERTEX,WEIGHT>:: 
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    CVI cv1, cv2;
    VI v1, v2;
    if ( !(_v1 == _v2) && IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        if ( !IsEdge(_v1,_v2) ) {
            v1 = const_cast<VI>(cv1);
            v2 = const_cast<VI>(cv2);
            v1->AddEdge(v2->vid,_weight);
            v2->AddEdge(v1->vid,_weight);
            this->numEdges++;
            return 0;
        } else {
#ifndef QUIETGRAPH
            cout << "\nIn AddEdge: edge already in graph, not added";
#endif
            return 1;
        }
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT> _wtpair) {
    CVI cv1, cv2;
    VI v1, v2;
    if ( !(_v1 == _v2) && IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) ) {
        if ( !IsEdge(_v1,_v2) ) {
            v1 = const_cast<VI>(cv1);
            v2 = const_cast<VI>(cv2);
            v1->AddEdge(v2->vid,_wtpair.first);
            v2->AddEdge(v1->vid,_wtpair.second);
            this->numEdges++;
            return 0;
        } else {
#ifndef QUIETGRAPH
            cout << "\nIn AddEdge: edge already in graph, not added";
#endif
            return 1;
        }
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}



template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, EI _ei) {
    CVI cv1, cv2;
    VI v1, v2;
    VID v2id = _ei->vertex2id;
    WEIGHT weight = _ei->weight;
    
    if (_v1id != v2id && IsVertex(_v1id,&cv1) && IsVertex(v2id,&cv2) ) {
        if ( !this->IsEdge(_v1id,v2id) ) {
            v1 = const_cast<VI>(cv1);
            v2 = const_cast<VI>(cv2);
            v1->AddEdge(v2id,weight);
            v2->AddEdge(_v1id,weight);
            this->numEdges++;
            return 0;
        } else {
#ifndef QUIETGRAPH
            cout << "\nIn AddEdge: edge already in graph, not added";
#endif
            return 1;
        }
    } else {
        cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
        return 1;
    }
}


template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteEdge(VID _v1id, VID _v2id, int _n) {
    CVI cv1, cv2;
    VI v1, v2;
    
    if ( IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) &&this->IsEdge(_v1id,_v2id) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        int ok1 = v1->DeleteXEdges(_v2id,-1);
        int ok2 = v2->DeleteXEdges(_v1id,-1);
        if ( ok1==1 && ok2==1 ) { //each should have found only 1 
            this->numEdges--;
            return 0;
        } 
    }
    return 1;
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _w, int _n) {
    CVI cv1, cv2;
    VI v1, v2;
    
    if ( IsVertex(_v1id,&cv1) && IsVertex(_v2id,&cv2) &&this->IsEdge(_v1id,_v2id,_w) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        int ok1 = v1->DeleteXEdges(_v2id,-1);
        int ok2 = v2->DeleteXEdges(_v1id,-1);
        if ( ok1==1 && ok2==1 ) { //each should have found only 1 
            this->numEdges--;
            return 0;
        } 
    }
    return 1;
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n) {
    CVI cv1, cv2;
    VI v1, v2;
    
    if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) &&this->IsEdge(_v1,_v2) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        int ok1 = v1->DeleteXEdges(v2->vid,-1);
        int ok2 = v2->DeleteXEdges(v1->vid,-1);
        if ( ok1==1 && ok2==1 ) { //each should have found only 1
            this->numEdges--; 
            return 0;
        }
    }
    return 1;
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w, int _n) {
    CVI cv1, cv2;
    VI v1, v2;
    
    if ( IsVertex(_v1,&cv1) && IsVertex(_v2,&cv2) &&this->IsEdge(_v1,_v2,_w) ) {
        v1 = const_cast<VI>(cv1);
        v2 = const_cast<VI>(cv2);
        int ok1 = v1->DeleteXEdges(v2->vid,-1);
        int ok2 = v2->DeleteXEdges(v1->vid,-1);
        if ( ok1==1 && ok2==1 ) { //each should have found only 1
            this->numEdges--; 
            return 0;
        }
    }
    return 1;
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
ChangeEdgeWeight(VID _v1id, VID _v2id, WEIGHT _weight) {
    
    if ( DeleteEdge(_v1id, _v2id) == 0) {
        return AddEdge(_v1id, _v2id,_weight);
    } else {
        return 1;
    }  
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
ChangeEdgeWeight(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    
    if ( DeleteEdge(_v1, _v2) == 0) {
        return AddEdge(_v1, _v2,_weight);
    } else {
        return 1;
    }  
}


//==================================
// WeightedGraph class Methods: Getting Data & Statistics
//==================================

// only report each edge once
template<class VERTEX, class WEIGHT>
vector< pair< pair<VID,VID>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetEdges() const {
    vector< pair< pair<VID,VID>, WEIGHT> > edges;
    
    edges.reserve(this->numEdges);
    
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
        for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
            if ( vi->vid < ei->vertex2id) {
                pair<VID,VID> newedge(vi->vid, ei->vertex2id);
                pair<pair<VID,VID>,WEIGHT> newedgewt(newedge, ei->weight);
                edges.push_back( newedgewt );
            }
        }
    }
    return edges;
}

// only report each edge once
template<class VERTEX, class WEIGHT>
vector< pair< pair<VERTEX,VERTEX>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetEdgesVData() const {
    vector< pair< pair<VERTEX,VERTEX>, WEIGHT> > edges;
    
    edges.reserve(this->numEdges);
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
        for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
            if ( vi->vid < ei->vertex2id) {
                VERTEX v2data = GetData(ei->vertex2id);
                pair<VERTEX,VERTEX> newedge(vi->data, v2data);
                pair<pair<VERTEX,VERTEX>,WEIGHT> newedgewt(newedge, ei->weight);
                edges.push_back( newedgewt );
            }
        }
    }
    return edges;
}


template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
GetVertexDegree(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return v1->edgelist.size(); 
    } else {
        cout << "\nGetVertexDegree: vertex "<< _v1id << " not in graph";
        return 1;
    }
}

template<class VERTEX, class WEIGHT>
vector<VID>
WeightedGraph<VERTEX,WEIGHT>::
GetAdjacentVertices(VID _v1id) const {
    return this->GetSuccessors(_v1id);
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VID,VID>,WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetIncidentEdges(VID _v1id) const {
    vector< pair<pair<VID,VID>,WEIGHT> > iedges;
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        iedges.reserve( v1->edgelist.size() );
        for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            pair<VID,VID> nextedge(_v1id,ei->vertex2id);
            pair<pair<VID,VID>,WEIGHT> nextedgewt(nextedge,ei->weight);
            iedges.push_back( nextedgewt );
        }
    } else {
        cout << "\nGetIncidentEdges: vertex "<< _v1id << " not in graph";
    }
    return iedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VERTEX,VERTEX>,WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetIncidentEdgesVData(VID _v1id) const {

     vector< pair<pair<VERTEX,VERTEX>,WEIGHT> > iedges;
     CVI v1;
     if ( IsVertex(_v1id,&v1) ) {
         iedges.reserve( v1->edgelist.size() );
         for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
            pair<VERTEX,VERTEX> nextedge( this->GetData(_v1id), this->GetData(ei->vertex2id));
            pair<pair<VERTEX,VERTEX>,WEIGHT> nextedgewt(nextedge,ei->weight);
            iedges.push_back( nextedgewt );
        }
    } else {
        cout << "\nGetIncidentEdgesVData: vertex "<< _v1id << " not in graph";
    }
    return iedges;
}


//==================================
// WeightedGraph class Methods: Connected Components Utilities
//==================================

template<class VERTEX, class WEIGHT>
bool
WeightedGraph<VERTEX,WEIGHT>::
IsSameCC (VID _v1id, VID _v2id) const {
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = this->BFS(_v1id); 
    
    if ( bfstree.IsVertex(_v1id) && bfstree.IsVertex(_v2id) ) {
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool
WeightedGraph<VERTEX,WEIGHT>::
IsSameCC (VERTEX& _v1, VERTEX& _v2) const {
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = this->BFS(_v1);
    
    if ( bfstree.IsVertex(_v1) && bfstree.IsVertex(_v2) ) {
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
vector<VID> 
WeightedGraph<VERTEX,WEIGHT>::
GetCC ( VID _v1id) const {
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = this->BFS(_v1id);
    vector<VID> ccverts = bfstree.GetVerticesVID();
    return ccverts;
}

template<class VERTEX, class WEIGHT>
vector<VERTEX>
WeightedGraph<VERTEX,WEIGHT>::
GetCC ( VERTEX& _v1) const {
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = this->BFS(_v1);
    vector<VERTEX> ccverts = bfstree.GetVerticesData();
    return ccverts;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VID,VID>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdges ( VID _v1id) const {
    
    vector< pair<pair<VID,VID>,WEIGHT> > ccedges, newedges;
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = this->BFS(_v1id);
    vector<VID> ccverts = bfstree.GetVerticesVID();
    for (unsigned int i=0; i < ccverts.size(); i++) {
        newedges = GetIncidentEdges(ccverts[i]);
        for (int k=0; k < newedges.size(); k++){
            ccedges.push_back ( newedges[k] );
        }
    }
    return ccedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VID,VID>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdges ( VERTEX&  _v1) const {
    return GetCCEdges ( GetVID(_v1) );
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VERTEX,VERTEX>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdgesVData ( VID  _v1id) const {
    
    vector< pair<pair<VERTEX,VERTEX>,WEIGHT> > ccedges, newedges;
    
    WeightedMultiDiGraph<VERTEX,WEIGHT> bfstree = this->BFS(_v1id);
    vector<VID> ccverts = bfstree.GetVerticesVID();
    for (unsigned int i=0; i < ccverts.size(); i++) {
        newedges = GetIncidentEdgesVData(ccverts[i]);
        for (int k=0; k < newedges.size(); k++){
            ccedges.push_back ( newedges[k] );
        }
    }
    return ccedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<pair<VERTEX,VERTEX>, WEIGHT> >
WeightedGraph<VERTEX,WEIGHT>::
GetCCEdgesVData ( VERTEX&  _v1) const {
    
    return GetCCEdgesVData ( GetVID(_v1) );
}

// return 2D vector ccedges[i,j] = jth edge of ith CC, edge is VERTEX pair
template<class VERTEX, class WEIGHT>
vector< vector< pair<VERTEX,VERTEX> > >
WeightedGraph<VERTEX,WEIGHT>::
GetEdgesByCCVDataOnly() const {
    
    vector< vector< pair<VERTEX,VERTEX> > > ccedges;
    
    vector< pair<int,VID> > cc = GetCCStats();
    for (unsigned int i=0; i < cc.size(); i++) {
        vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > thiscc = GetCCEdgesVData(cc[i].second);
        vector< pair<VERTEX,VERTEX> >  edges;
        for (int j=0; j < thiscc.size(); j++) {
            pair<VERTEX,VERTEX> thisedge(thiscc[j].first.first,thiscc[j].first.second);
            edges.push_back ( thisedge );
        }
        ccedges.push_back ( edges );
    }
    return ccedges;
}

template<class VERTEX, class WEIGHT>
vector< pair<int,VID> > 
WeightedGraph<VERTEX,WEIGHT>::
GetCCStats () const {
    vector< pair<int,VID> > ccstats;
    vector<VID> verts = this->GetVerticesVID();
    
    while ( verts.size() != 0 ) {
        VID v1id = verts.front();
        vector<VID> CC = GetCC(v1id);
        int CCsize = CC.size();
        ccstats.push_back( pair<int,VID>(CCsize,v1id) );
        sort( CC.begin(), CC.end() ); //sort by VID for set_difference
        set_difference
            ( verts.begin(),verts.end(),CC.begin(),CC.end(),verts.begin() );
        verts.erase(verts.end()-CC.size(), verts.end());
    }
    
///Modified for VC
#if defined(WIN32)
    typedef bool (*Compare_Fun_Ptr)(const pair<int,VID>& , const pair<int,VID>&);
    sort ( ccstats.begin(), ccstats.end(), ptr_fun( (Compare_Fun_Ptr)CCVID_Compare ) );
#else
    sort ( ccstats.begin(), ccstats.end(), ptr_fun( CCVID_Compare ) );
#endif
    
    return ccstats;
}

template<class VERTEX, class WEIGHT>
int
WeightedGraph<VERTEX,WEIGHT>::
GetCCcount () const {
    vector< pair<int,VID> > ccstats = GetCCStats();
    return ccstats.size();
}


//==================================
// WeightedGraph class Methods: Display, Input, Output 
//==================================


template<class VERTEX, class WEIGHT>
void
WeightedGraph<VERTEX,WEIGHT>::
DisplayCC ( VID _v1id) const {
    
    typedef vector<VID>::iterator VI;
    
    vector<VID> ccverts = GetCC(_v1id);
    cout << "\nCC[" << _v1id << "] = {";
    for (VI vi = ccverts.begin(); vi < ccverts.end(); vi++ ) {
        cout << *vi; 
        if (vi != ccverts.end() -1 ) cout << ", ";
    }
    cout << "}\n";
}

template<class VERTEX, class WEIGHT>
void
WeightedGraph<VERTEX,WEIGHT>::
DisplayEdgesByCCVDataOnly() const {
    
    
    vector< vector< pair<VERTEX,VERTEX> > >  ccedges = GetEdgesByCCVDataOnly();
    
    cout << endl << "Edges in each connected component (vertex data shown)";
    for (int cc=0; cc < ccedges.size(); cc++){
        cout << endl << "CC[" << cc << "]: ";
        for (int e=0; e < ccedges[cc].size(); e++){ 
            cout << " (" << ccedges[cc][e].first << "," << ccedges[cc][e].second << ")"; 
        }
    }
    
}

template<class VERTEX, class WEIGHT>
void
WeightedGraph<VERTEX,WEIGHT>::
DisplayCCStats(int _maxCCprint) const {
    
    ///Modified for VC
    typedef vector< pair<int,VID> > PAIR_VECTOR;
    typedef PAIR_VECTOR::const_iterator CCI; 
    
    int maxCCprint;
    
    vector< pair<int,VID> > ccstats = GetCCStats();
    if (_maxCCprint == -1) {
        maxCCprint = ccstats.size();
    } else {
        maxCCprint = _maxCCprint;
    }
    
    int ccnum = 1;
    cout << "\nThere are " << ccstats.size() << " connected components:";
    for (CCI vi = ccstats.begin(); vi != ccstats.end(); vi++) {
        cout << "\nCC[" << ccnum << "]: " << vi->first ; 
        cout << " (vid=" << vi->second << ")";
        ccnum++;
        if (ccnum > maxCCprint) return; 
    }
}

//==================================
// WeightedGraph class Predicates, Comparisons & Operations
//==================================
template<class VERTEX, class WEIGHT>
bool
#ifdef __HP_aCC
WeightedGraph<VERTEX,WEIGHT>::CCVID_Compare(const pair<int,VID> _cc1, const pair<int,VID> _cc2)
#else 
WeightedGraph<VERTEX,WEIGHT>::CCVID_Compare(const pair<int,VID>& _cc1, const pair<int,VID>& _cc2)
#endif
{
    return (_cc1.first > _cc2.first ) ;
}


//================================
// WeightedGraph class Methods: Transform DAG to Undirected
//==================================
//  WeightedGraph<VERTEX,WEIGHT>
//      DagToUndirected(WeightedMultiDiGraph<VERTEX, WEIGHT>&);
//==================================

template<class VERTEX, class WEIGHT>
WeightedGraph<VERTEX,WEIGHT>
WeightedGraph<VERTEX,WEIGHT>:: 
DagToUndirected(WeightedMultiDiGraph<VERTEX, WEIGHT>& _dag) {
    VI v1,v2;
    CVI cv2;
    WeightedGraph<VERTEX,WEIGHT> ung;
    
    for(v1=_dag.v.begin(); v1<_dag.v.end(); v1++) { 
        VID _v1id = v1->vid;
        ung.AddVertex(v1->data,_v1id);
    }
    
    for(v1=_dag.v.begin(); v1<_dag.v.end(); v1++) { 
        VID _v1id = v1->vid;
        for(EI e1=v1->edgelist.begin(); e1<v1->edgelist.end(); e1++) {
            VID _v2id = e1->vertex2id;
            ung.AddEdge(_v1id, _v2id, e1->weight);
        }
    }
    
    ung.SetnumVerts( _dag.GetVertexCount() );
    return ung;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template WtVertexType Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//==================================
// WtVertexType class Methods: Constructors and Destructor
//==================================

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(){
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(VERTEX& _data, VID _id){
    data = _data;
    vid = _id;
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(VERTEX& _data, VID _id, int _edgelistsz){
    data = _data;
    vid = _id;
    edgelist.reserve( _edgelistsz );
}


template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
~WtVertexType(){
}

//==================================
// Vertex class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdge(VID _v2id, WEIGHT _weight) {
     WtEdge newEdge(_v2id, _weight);
     edgelist.push_back(newEdge);
}

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdgewCheck(VID _v2id, WEIGHT _weight) {
  bool found = false;
  for(EI ei=edgelist.begin(); ei!=edgelist.end(); ei++) {
    if(_v2id == ei->vertex2id) found=true;
  }
  if(!found) {
     WtEdge newEdge(_v2id, _weight);
     edgelist.push_back(newEdge);
  }

}

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>:: 
AddPredecessorEdge(VID _v0id, WEIGHT _weight) {
     WtEdge newEdge(_v0id, _weight);
     predecessors.push_back(newEdge);
}


//delete upto _x edges (v1,v2) of any weight (delete first encountered)
template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
DeleteXEdges(VID _v2id, int _x) {
    int num_to_delete; 
    int num_deleted = 0;
    if (_x == -1) 
        num_to_delete = edgelist.size();
    else num_to_delete = _x;
    EI ei = my_find_EID1_eq(_v2id);
    while ( (ei != edgelist.end()) && (num_deleted < num_to_delete) ) {
        edgelist.erase(ei);
        num_deleted++;
        ei = my_find_EID2_eq(ei,edgelist.end(),_v2id);
    }
    return num_deleted;
}

//delete upto _x edges (v1,v2) of specified weight (delete first encountered) 
template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
DeleteXEdges(VID _v2id, WEIGHT _weight, int _x) {
    int num_to_delete;
    int num_deleted = 0;
    if (_x == -1)
        num_to_delete = edgelist.size();
    else num_to_delete = _x;
    EI ei = my_find_EID1_eq(_v2id);
    while ( (ei != edgelist.end()) && (num_deleted < num_to_delete) ) {
        if (ei->weight == _weight) {
            edgelist.erase(ei);
            num_deleted++;
            ei = my_find_EID2_eq(ei,edgelist.end(),_v2id);
        } else {
            ei = my_find_EID2_eq(ei+1,edgelist.end(),_v2id);
        }
    }
    return num_deleted;
}


//==================================
// Vertex class Methods: Finding Edges
//==================================

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id) const {
    EI ei;
    return ( IsEdge(_v2id, &ei) );
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, const WtEdge** _ei) const {
    CEI ei = my_find_EID1_eq(_v2id);
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdge(VID _v2id, WEIGHT _weight, const WtEdge** _ei) const {
    CEI ei = my_find_EIDWT_eq( pair<VID,WEIGHT>(_v2id,_weight) );
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}

//==================================
// Vertex class Methods: Getting Data & Statistics 
//==================================

template<class VERTEX, class WEIGHT>
VERTEX 
WtVertexType<VERTEX,WEIGHT>::
GetData() const {
    return data;
}

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
GetVID() const {
    return vid;
}

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeCount() const {
    return edgelist.size();
}

template<class VERTEX, class WEIGHT>
WEIGHT 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeWeight(VID _v2id) const {
    EI ei;
    CEI cei;
    if (IsEdge(_v2id,&cei)) {
        ei = const_cast<EI> (cei);
        return ei->weight;
    } else {
        cout << "\nGetEdgeWeight: edge not in graph";
        return WEIGHT::InvalidWeight();
    }
}

//==================================
// Vertex class Methods: Display, Input, Output 
//==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>::
DisplayEdgelist() const {
	/*
    cout << "id =" << setw(3) << vid << ", data = ["; 
    cout << data;
    cout << "], edges={";
    for (CEI ei = edgelist.begin(); ei < edgelist.end(); ei++) {
        ei->DisplayEdge();
        if (ei != edgelist.end() - 1) cout << ", ";
    }
    cout << "} \n";
	*/
}

template<class VERTEX, class WEIGHT>
void
WtVertexType<VERTEX,WEIGHT>::
WriteEdgelist(ostream& _myostream) const {
    
    _myostream << vid << " "; 
    _myostream << data << " "; 
    _myostream << edgelist.size() << " "; 
    
    for (CEI ei = edgelist.begin(); ei != edgelist.end(); ei++) { 
        ei->WriteEdge(_myostream);
        _myostream << " "; 
    }
}

//==================================
// Vertex class Predicate Utilities
//==================================

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EID1_eq(const EID _eid) const {
    CEI ei = edgelist.begin();
    bool found = false;
    while (ei != edgelist.end() && !found) {
        if ( ei->vertex2id == _eid) {
            found = true;
        } else {
            ei++;
        }
    }
    return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EID2_eq(const WtEdgeType<VERTEX,WEIGHT>* _start, 
                const WtEdgeType<VERTEX,WEIGHT>* _end,
                const EID _eid) const {
    CEI ei = _start;
    bool found = false;
    while (ei != _end && !found) {
        if ( ei->vertex2id == _eid) {
            found = true;
        } else {
            ei++;
        }
    }
    return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EWT_eq(const WEIGHT _wt) const { 
    CEI ei = edgelist.begin();
    bool found = false;
    while (ei != edgelist.end() && !found) {
        if ( ei->weight == _wt) {
            found = true;
        } else {
            ei++;
        }
    }
    return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>*
WtVertexType<VERTEX,WEIGHT>::
my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair) const {
   CEI cei = edgelist.begin();
   EI ei = const_cast<EI> (cei);
   
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->vertex2id==_wtpair.first && ei->weight == _wtpair.second ) {
         found = true;
      } else {
         ei++;
      }
   }
   return const_cast<WtEdgeType<VERTEX,WEIGHT>*>(ei);
}


/*--------------- NMA: these don't work with sgi/CC, but do for sun/g++
template<class VERTEX, class WEIGHT>
class 
WtVertexType<VERTEX,WEIGHT>::
EID_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
public:
explicit EID_eq(const VID i) : testid (i) {}
bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
return e.vertex2id == testid;
}
VID testid;
protected:
private:
}

  
    template<class VERTEX, class WEIGHT>
    class 
    WtVertexType<VERTEX,WEIGHT>::
    EWT_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
    public:
    explicit EWT_eq(const WEIGHT w) : testwt (w) {}
    bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
    return e.weight == testwt;
    }
    WEIGHT testwt;
    protected:
    private:
    }
    
      template<class VERTEX, class WEIGHT>
      class 
      WtVertexType<VERTEX,WEIGHT>::
      EIDWT_eq : public unary_function< WtEdgeType<VERTEX,WEIGHT>,bool> {
      public:
      explicit EIDWT_eq(const pair<VID,WEIGHT> eid) : testedge (eid) {}
      bool operator() (WtEdgeType<VERTEX,WEIGHT> e) {
      return ((e.vertex2id==testedge.first) && (e.weight == testedge.second));
      }
      pair<VID,WEIGHT> testedge;
      protected:
      private:
      }
*/

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template WtEdge Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//==================================
// WtEdge class Methods: Constructors and Destructor
//==================================

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(){
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(VID _v2id, WEIGHT _weight){
    vertex2id = _v2id;
    weight = _weight;
}

template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
~WtEdgeType(){
}

//==================================
// WtEdge class Methods: Getting Data & Statistics
//==================================

//==================================
// WtEdge class Methods: Display, Input, Output
//==================================

template<class VERTEX, class WEIGHT>
void 
WtEdgeType<VERTEX,WEIGHT>:: 
DisplayEdge() const {
    cout << vertex2id << "(" << weight << ")";
} 

template<class VERTEX, class WEIGHT>
void
WtEdgeType<VERTEX,WEIGHT>::
WriteEdge(ostream& _myostream) const {
    _myostream << vertex2id << " " << weight << " ";
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  The End
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////
#endif

// $Id: BaseGraph.h,v 1.2 2004/08/17 22:05:13 neilien Exp $

/////////////////////////////////////////////////////////////////////
/**@mainpage  Graph Library
 *
 *@section gen General Description
 *     This set of template classes provides an implementation for 
 *     the graph abstract data type. It is recomended that the following
 *     document will be read first:
 *     <a href ="https://blackwidowers.cs.tamu.edu/stapldoc/graph/newgraph.ps">[ps]</a><a href ="https://blackwidowers.cs.tamu.edu/stapldoc/graph/newg/index.html">[html]</a>
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
 * @date 04/02/02  
 * @author Gabriel Tanase(Class hierarchy and algorithms)<p>
 * Paul Thomas(Algorithms(Dijkstra) and testing)<p>
 * Ping An (Involved in the design)<p>
 * Based on original Graph.h written by Nancy Amato
 */
/////////////////////////////////////////////////////////////////////

#ifndef BaseGraph_h
#define BaseGraph_h

////////////////////////////////////////////////////////////////////////////////////////////
//include standard headers

#include "Defines.h"

#ifdef _PGRAPH
#include <runtime.h>
#include "rmitools.h"
#endif

namespace graph{

#ifndef VID
///ID for every vertex in graph.
typedef int VID;
#endif

////////////////////////////////////////////////////////////////////////////////////////////


#ifndef EID
typedef short EID;
#endif

#ifndef INVALID_VID
#define INVALID_VID    -999
#endif

typedef int COLOR;


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
  *    first endpoint is implicit due to the adjacency list rep) 
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
#ifdef _PGRAPH
       WtEdgeType(VID,WEIGHT,int);
#endif
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
        void DisplayEdge(int wt) const;
        ///Out put Id of the second endpoint and weight to the given output stream
        void WriteEdge(ostream&, int wt) const;
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

#ifdef _PGRAPH
    int edgeid;
    int GetEdgeId(){
      return edgeid;
    }

    int GetEdgeId() const {
      return edgeid;
    }

    void SetEdgeId(int __id){
      edgeid = __id;
    }

    void define_type(stapl::typer &t)  
    {
      t.local(vertex2id);
      t.local(weight);
      t.local(edgeid);
    }
#endif

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

     //Modified for VC
    /**This seems VC is not be able to use vector<>::iterator directly without
      *"using namespace std;". how every this causes bigger problem.
      *therefore, one more typedef is used here
      */
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
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
  

  WtVertexType();
  WtVertexType(const VERTEX&, VID);       // don't reserve space for edges
  WtVertexType(const VERTEX&, VID, int);  // 'reserve' space for edges
  ~WtVertexType();
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
    *Return the success of the operation(Graph) / edge identifier(pGraph)
        */
      int AddEdge(VID, WEIGHT); 

      /**Adding a edge by given id for the second endpoint and the weight of 
        *this new edge. Check if the edge is already added, if yes, do nothing.
    *Return the success of the operation(Graph) / edge identifier(pGraph)
        */
      int AddEdgewCheck(VID, WEIGHT); 

      /**Adding a predecessor edge by given id for the first endpoint and the weight of 
        *this new edge.
        */
    void AddPredecessorEdge(VID, WEIGHT);

      /**Delete edge(s) whose endpoint is given.
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

#ifdef _PGRAPH
      /**
       *For pGraph. Fix the coment
       */
      int DeleteEdgeId(int);
#endif

      /**Check if any edge having this given vid.
        *return true if this kind of edge exists in edgelist.
        */
      bool IsEdge(VID) const ; 

      /**Check if any edge having this given vid and return the address of this edge.
        *return true if this kind of edge exists in edgelist.
        *@param CEI the iterator of edge with given VID. If there are more
        *than one edges, this will be the first one found in the list.
        */
      bool IsEdge(VID, CEI*) const;

      /**Check if any edge having this given vid and return the address of this edge.
        *return true if this kind of edge exists in edgelist.
        *@param CEI the iterator of the edge with given VID. If there are more
        *than one edges, this will be the first one found in the list.
        */
      bool IsEdge(VID, EI*);


      /**Check if any edge having this given vid and weight 
        *the address of this edge  will be returned.
        *return true if this kind of edge exists in edgelist.
        *@param WEIGHT qualifier
        *@param CEI the iterator of the edge with given VID. If there are more
        *than one edges, this will be the first one found in the list.
        */
      bool IsEdge(VID, WEIGHT, CEI* _ei) const;

      /**Check if any edge having this given vid and weight 
        *the address of this edge  will be returned.
        *return true if this kind of edge exists in edgelist.
        *@param WEIGHT qualifier
        *@param EI the iterator of the edge with given VID. If there are more
        *than one edges, this will be the first one found in the list.
        */
      bool IsEdge(VID, WEIGHT, EI* _ei) ;

#ifdef _PGRAPH
      bool IsEdgeId(int, CEI*) const;
      bool IsEdgeId(int, EI*);
#endif

      //////////////////////////////////////////////////////////////////////////////////////////
      //
      //    Getting Data & Statistics
      //
          //////////////////////////////////////////////////////////////////////////////////////////

      ///Get User specified data stored in this vertex.
      //!!!!!!! gty
      VERTEX&  GetData() ; 
      const VERTEX&  GetData() const; 
      VERTEX&  GetUserData() ; 
      const VERTEX&  GetUserData() const; 
      void SetUserData(const VERTEX& _data);
      void SetUserData(VERTEX& _data);
      ///Get VID of this instance of WtVertexType.
      int  GetVID() const;
      ///Number of edges connected
      int  GetEdgeCount() const;
      /**Get weight of edge whose second endpoint has VID
        *@param VID the id for looking for edge.
        *@return the first edge having this id was returned.
        */
      WEIGHT  GetEdgeWeight(VID) const; 

#ifdef _PGRAPH
      WEIGHT GetEdgeIdWeight(int) const ;
      EI  find_edgeid_eq(const EID);
      CEI find_edgeid_eq(const EID) const;
#endif

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
      *The only parameter specify if the graph is weighted or not
      *and depending on this we will prin the weights or not
          *@note this print out some kind of human readable data.
          *@see WriteEdgelist
          */
        void DisplayEdgelist(int wt) const;

       /**Output information about this vertex to given output stream.
          *Print out vertex id, user specified data, and every edge.
          *The wt parameter specify if the graph is weighted or not
      *and depending on this we will prin the weights or not
          *@note this print out some kind of NON-human-readable data.
          *format: VID VERTEX #edges VID WEIGHT VID WEIGHT ... 
          *@see DisplayEdgelist
          */
        void WriteEdgelist(ostream&,int wt) const;
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

    // Predicates

    /**check if _eid is an edge in the list, edgelist.
      *@return a constant edge iterator to the vertex which has this ID
      */
    CEI my_find_EID1_eq(const EID _eid) const; 
    /**check if _eid is an edge in the list, edgelist.
      *@return an edge iterator to the vertex which has this ID
      */
    EI my_find_EID1_eq(const EID _eid); 

    /**check if _eid is an edge in the list starting from _start to _end.
      *@return a constant edge iterator to the vertex which has this ID
      */
    CEI my_find_EID2_eqc(CEI _start, CEI _end, const EID _eid) const; 

    /**check if _eid is an edge in the list starting from _start to _end.
      *@return an edge iterator to the vertex which has this ID
      */
    EI my_find_EID2_eq(EI _start, EI _end, const EID _eid) ; 


    /**check if any edge in the list, edgelist, has weight, _wt.
      *@return WtEdge which has this weight
      */
    CEI my_find_EWT_eq(const WEIGHT _wt) const; 

    /**check if any edge in the list, edgelist, has weight, _wtpair.second
      *and ID, _wtpair.first
      *@return a constant edge iterator to the edge which has this weight
      */
    CEI my_find_EIDWT_eqc(const pair<VID,WEIGHT> _wtpair) const;

    /**check if any edge in the list, edgelist, has weight, _wtpair.second
      *and ID, _wtpair.first
      *@return a constant edge iterator to the edge which has this weight
      */
    EI my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair) ;


  //@}

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
  vector< WtEdge > predecessors; ///< call BaseGraph::SetPredecessors() to initialize

  
#ifdef _PGRAPH
  ///counter for edge ids
  
  int edgeid_cnt;//this is initialized with zero and is autoincremented

  //return edge id and increment the counter;
  int GetNextEdgeId(){
    int temp = edgeid_cnt;
    edgeid_cnt++;
    return temp;
  }

  void SetEdgeIdCnt(int _cnt){
    edgeid_cnt = _cnt;
  }

  //for rmi use only, not for sequential graph
  vector<COLOR> in;
  vector<COLOR> out;
  /**
   *Sets the maximum number of symultaneous traversals
   *@return the previous number of traversals
   */
  int SetTraversalNumber(int _tn){
    int temp = in.size();
    in.resize(_tn);
    out.resize(_tn);
    return temp;
  }
  COLOR GetColorIn(int _tr_id){
    assert(_tr_id < in.size());
    return in[_tr_id];
  }
  COLOR GetColorOut(int _tr_id){
    assert(_tr_id < out.size());
    return out[_tr_id];
  }
  COLOR SetColorIn(int _tr_id,COLOR _color){
    assert(_tr_id < in.size());
    return in[_tr_id]=_color;
  }
  COLOR SetColorOut(int _tr_id,COLOR _color){
    assert(_tr_id < out.size());
    //if(vid == 4994 || vid== 5016 || vid== 5038 || vid==5060){
    //  cout<<"somebody touching"<<vid<<": to set the color "<<_color<<":"<<stapl::get_thread_id()<<endl;
    //}
    return out[_tr_id]=_color;
  }
  
  void ResetTraversals(){
    for(int i=0;i<in.size();i++){
       in[i]=0;
      out[i]=0;
    }
  }
  void ResetTraversal(int _tr_id){
     in[_tr_id]=0;
    out[_tr_id]=0;
  }

  void SetDependencies(){
    for(int i=0;i<in.size();i++){
       in[i] = predecessors.size();
      out[i] = edgelist.size();
    }
  }
  
  void SetDependencies(int _tr_id){
     in[_tr_id] = predecessors.size();
    out[_tr_id] = edgelist.size();
    
    //if(vid == 4994 || vid== 5016 || vid== 5038 || vid==5060){
    //  cout<<"INSIDE::"<<_tr_id<<":"<<vid<<":"<< predecessors.size()<<":"<< edgelist.size()<<endl;
    //}
  }

  int ByteSize(){
    int sz = 4;
    sz += sizeof(data) + sizeof(vid);
    if (edgelist.size() > 0)
      sz += edgelist.size() * sizeof(edgelist[0]);
    
    //cout<<"EDGELISTSIZE:"<<sizeof(edgelist[0])<<":"<<sizeof(float)<<endl;
    

    if (predecessors.size() > 0)
      sz += predecessors.size() * sizeof(predecessors[0]);
    if (in.size() > 0)
      sz += in.size() * sizeof(in[0]);
    if (out.size() > 0)
      sz += out.size() * sizeof(out[0]);

    sz += sizeof(edgeid_cnt);

    return sz;
  }

  void define_type(stapl::typer &t)  
    {
      t.local(data);
      t.local(vid);
      t.local(edgelist);
      t.local(predecessors);

      //added for traversals
      t.local(in);
      t.local(out);
      t.local(edgeid_cnt);
    }
#endif
  
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


//don't relocate this statement
//#ifdef _PGRAPH
//#include "../pContainers/pgraph/pVertex.h"
//#endif


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
       virtual ~AbstractGraph();

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
       //virtual bool IsEdge(VID, VID) const = 0; 
       ///Abstract
       virtual bool IsVertex(VERTEX&)  const = 0;
       ///Abstract
       //virtual bool IsEdge(VERTEX&, VERTEX&) const  = 0; 

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
       
       /*      ///Abstract */
     /*        virtual void DisplayGraph() const = 0;  */
     /*        ///Abstract */
     /*        virtual void DisplayVertexAndEdgelist(VID) const = 0;  */
     
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
   int directness;

#ifdef _PGRAPH
    void define_type(stapl::typer &t)
    {
      t.local(this->vertIDs);
      t.local(this->numVerts);
      t.local(this->numEdges);
      t.local(this->reserveEdgesPerVertex);
      t.local(this->directness);
    }
#endif


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
public:
    inline void SetNumEdges(int _ne){
      numEdges = _ne;
    }
    inline int GetNumEdges() const{
      return numEdges;
    }
    inline void SetNumVerts(int _ne){
      numVerts = _ne;
    }
    inline int GetNumVerts() const {      
      return numVerts;
    }

   inline int DecrementVerts(int _nv){
     numVerts -= _nv;
     return numVerts;
   }
   
   inline int IncrementVerts(int _nv){
     numVerts += _nv;
     return numVerts;
   }
   
   inline int DecrementEdges(int _ne){
     numEdges -= _ne;
     return numEdges;
   }
   
   inline int IncrementEdges(int _ne){
     numEdges += _ne;
     return numEdges;
   }
   inline VID getVertIDs() const {
     return vertIDs;
   }
   inline int setVertIDs(VID _v){
     if(_v < vertIDs) {
       printf("ERROR: You are trying to set as max vid ");
       printf("something smaller than current one\n");
       //????? should I return ERROR??????
     }
     vertIDs = _v;
     return OK;
   }
};



/**Derived from AbstractGraph<VERTEX,WEIGHT>.
  *
  *The BaseGraph class:
  * -# is a *directed* weighted graph
  * -# allows multiple vertices with the same VERTEX data
  * -# allows multiple (v1id,v2id) edges with *different* WEIGHTs
  * -# allows multiple (v1id,v2id) edges with *same* WEIGHT
  *
  *The graph is represented by an adjacency list structure.
  *
  */


//The following two functions/classes are usef in SimpleSort of the graph

template<class T> class comparebyvid
{
public:
        bool operator () (T a, T b)
        {
                return a.vid > b.vid;
        }
};  
    
template<class T> class comparebypid
{
public:
        bool operator () (T a, T b) 
        {
                return a.pid <= b.pid;
        }
};




template<class VERTEX, class WEIGHT>
class BaseGraph :  
#ifdef _TESTBGRAPH
public BasePObject, 
#endif
public AbstractGraph<VERTEX> {
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

       /**Constructor. Do nothing.
         *@note this constrcutor didn't  reserve any space for verts or edges
         */
       BaseGraph();

       /**Constructor. 'reserve' space for vertices.
         *@param int how many vertices will be reserved.
         */
       BaseGraph(int);

       /**Constrcutor. 'reserve' space for vertices.
         *@param first_int how many vertices will be reserved.
         *@param first_int how many edges will be reserved.
         */
       BaseGraph(int,int);

       /**Destrcutor. Do nothing.
         */
       ~BaseGraph();

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


       int DeleteVertices(vector<VID>&); 

#ifdef DYNAMIC_GRAPH
       int UpdateMap();
#endif 

       /**Remove all vertices added to this graph and set every thing 
         *to zero, NULL, and false.
         *@return OK
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
         *ERROR will be returned.
         *@return OK if vid1 and vid2 are found in this graph and new edge
         *is created correctly
         */
       int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);

       /**Add an edge, from the v1 to v2 vertex with weight.
         *@param vid1 the id for the first vertex
         *@param vid2 the id for the second vertex
         *@param WEIGHT the weight for this new edge
         *@note vid1 and v2 should be added in graph before, otherwise
         *ERROR will be returned.
         *@return OK if vid1 and vid2 are found in this graph and new edge
         *is created correctly
         */
       int  AddEdge(VID, VID, WEIGHT);

       /**Add two edges, from the v1 to v2 vertex and 
         *from the v2 to v1 vertex.
         *@param v1 the user data for the first vertex
         *@param v2 the user data for the second vertex
         *@note pair.first is weight for v1->v2 and pair.second
         *is weight for v2->v1
         *@note v1 and v2 should be found in graph, otherwise
         *ERROR will be returned. if there are more than one vertex
         *constain v1 or v2, then the first one found in the graph will
         *be used.
         *@return OK if v1 and v2 are found in this graph and new edge
         *is created correctly.
         */
       int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);

       /**Add an edge, from the v1 to v2 vertex with given weight.
         *@param v1 the user data for the first vertex
         *@param v2 the user data for the second vertex
         *@param WEIGHT the weight for this new edge
         *@note v1 and v2 should be found in graph, otherwise
         *ERROR will be returned. if there are more than one vertex
         *constain v1 or v2, then the first one found in the graph will
         *be used.
         *@return OK if v1 and v2 are found in this graph and new edge
         *is created correctly.
         */
       int  AddEdge(VERTEX&, VERTEX&, WEIGHT);  

       /**Delete n edges from vid1 to vid2.
         *Delete edges starting from vid1 to vid2.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@note vid1 an vid2 should be in this graph
         *@return ERROR if vid1 and/or vid2 are not found. OK if ok.
         */
       int  DeleteEdge(VID, VID, int n=-1); // default, delete all

       /**Delete n edges from v1 to v2.
         *Delete edges starting from v1 to v2.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@note v1 an v2 should be in this graph
         *@return ERROR if v1 and/or v2 are not found. OK if ok.
         */
       int  DeleteEdge(VERTEX&,VERTEX&, int n=-1); //default, delete all

       /**Delete n edges from vid1 to vid2 of specified weight.
         *Delete edges starting from vid1 to vid2 of specified weight.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@param WEIGHT the edges of this weught will be deleted
         *@note vid1 an vid2 should be in this graph
         *@return ERROR if vid1 and/or vid2 are not found. OK if ok.
         */
       int  DeleteWtEdge(VID, VID, WEIGHT, int n=-1); // default, delete all

         /**Delete n edges from v1 to v2 of specified weight.
         *Delete edges starting from v1 to v2 of specified weight.
         *Here v1 is the vertex constains user data in first parameter,
         *v2 is the vertex constains user data in second parameter.
         *if n is -1 then all edges will be deleted.
         *@param n number of edges will be delete.
         *@param WEIGHT the edges of this weught will be deleted
         *@note v1 an v2 should be in this graph
         *@return ERROR if v1 and/or v2 are not found. OK if ok.
         */
       int  DeleteWtEdge(VERTEX&,VERTEX&,WEIGHT, int n=-1); //default, delete all

       /**
    *For pGraph; Fix the coment
    */
       int DeleteEdgeId(VID,int);

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
       bool IsVertex(VERTEX&) const ;
       //bool IsEdge(VID, VID) const;
       //bool IsEdge(VERTEX&, VERTEX&) const;
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
       /**Get all the vertices ids from the graph
    *@param The vector were the ids will be stored; the method will clear this vector before filling in;
    *return the number of elements that were pushed in the vector
    */
       int GetVerticesVID(vector<VID>&) const;

       /**Get n VIDs starting with VID _vid.
      *return the number of elements that were pushed in the vector
      *@param The vector were the ids will be stored; the method will clear this vector before filling in;
       *@note if _vid is not found in this graph, then a empty vector will be returned,
       *and an error message will be output to standard output.
       */
       int GetVerticesVID(vector<VID>&, VID _vid,int n) const;
       /**Get all the user data from the graph
    *@param verts The vector were the data will be stored; the method will clear this vector before filling in;
    *return the number of data elements that were pushed in the vector
    */
       int GetVerticesData(vector<VERTEX>& ) const;

       /** get n user data starting with VID _vid.
    *@param verts The vector were the data will be stored; the method will clear this vector before filling in;
    *return the number of data elements that were pushed in the vector
    *@note if _vid is not found in this graph, then a empty vector will be returned,
    *and an error message will be output to standard output.
         */
       int GetVerticesData(vector<VERTEX>&, VID _vid,int n) const;

       /**Get user data associated with given VID.
         *If this vid is not found in graph, then Invalide data 
         *provided by VERTEX::InvalidData() will be returned.
         */
       inline VERTEX  GetData(VID) const;

       /**Get user data associated with VIDs, from _v1id to _v2id.
         *If _v1id and/or _v2id are not found in graph, then
         *an empty vector will be returned.
         *@note _v1id should be smaller than _v2id
         */
       ///Get a pointer of User specified data stored in this vertex.
       VERTEX* GetReferenceofData(VID); 
       
       /*
    *Return all edges that have _v as destination
    */
       void GetAllEdgesToV(VID _v,vector< pair<VID,WEIGHT> > &ToEdges);

       int GetData(vector<VERTEX>&, VID _v1id, VID _v2id) const;

       /**Get VID from given user data.
         *If not such vertex is found, then INVALID_VID will
         *be returned.
         */
       VID     GetVID(VERTEX&) const ;

       /**Put user data to the specified vertex.
        *If this specified vertex could not be found in graph,
        *then nothing will happen.
        *@see WtVertexType::data
        */
       void PutData(VID, VERTEX);

       /**Return the number of vertices in the graph
     */
       int GetVertexCount() const;

       /**Return the id that will be allocated to the next vertex to be inserted in graph
    */
       VID  GetNextVID() const;
       int  GetEdgeCount() const;

       void SetStartID(VID);
       void SetnumVerts(int);
       void SetnumEdges(int);
       

       virtual void DisplayGraph() const;
   //@}

   //@}

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


   //added because of the pGraph -----!!!!!!!
   //agree about the name for this
   typedef VERTEX value_type;
   typedef WEIGHT weight_type;
   typedef VI iterator;
   typedef CVI const_iterator;
   

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
       VID  AddVertex(const VERTEX&,VID);

       /**Create a new edge from vid to EI->vertex2id.
         *Acutelly, this edgae, EI, has been created, this method
         *just adds this edge to vid's edge list.
         *@return ERROR if there is no vertex of given VID in graph.
         *OK is every thing is fine.
         */
       int AddEdge(VID, EI);


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
         *If the answer is yes then CVI will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VID)
         */
       bool IsVertex(VID, CVI*) const;

       /**Check if given VID is in graph.
         *If the answer is yes then VI will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VID)
         */
       bool IsVertex(VID, VI*) ;

       /**Check if given v with specified user data is in graph.
         *If the answer is yes then CVI will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VERTEX&)
         */
       bool IsVertex(VERTEX&, CVI*) const;

       /**Check if given v with specified user data is in graph.
         *If the answer is yes then VI will be the pointer
         *pointing to this vertex.
         *
         *@return true if given VID is in graph. Otherwise return false.
         *@see IsVertex(VERTEX&)
         */
       bool IsVertex(VERTEX&, VI*) ;

       /**Check if there is any edge connected from vid1 to vid2.
         *If the answer is yes then VI will be the pointer
         *pointing to the vertex where the edge from, and EI
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         *@see IsEdge(VID, VID)
         */
       bool IsEdge(VID, VID) ;

       /**Check if there is any edge connected from vid1 to vid2.
         *If the answer is yes then VI will be the pointer
         *pointing to the vertex where the edge from, and EI
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         *@see IsEdge(VID, VID)
         */
       bool IsEdge(VID, VID, VI*, EI*) ;
       /**Check if there is any edge connected from v1 to v2.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *If the answer is yes then VI will be the pointer
         *pointing to the vertex where the edge from, and EI
         *will be the pointer pointing to this edge.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         *@see IsEdge(VERTEX&, VERTEX&)
         */
       bool IsEdge(VERTEX&, VERTEX&, VI*, EI*) ;

       /**Check if there is any edge connected from vid1 to vid2 of specified weight.
         *If the answer is yes then VI will be the pointer
         *pointing to the vertex where the edge from, and EI
         *will be the pointer pointing to this edge.
         *
         *@return false if vid1 or vid2 are not in graph, or
         *there is no edge from vid1 to vid2.
         */
       bool IsEdge(VID, VID, WEIGHT, VI*, EI*);

       /**Check if there is any edge connected from v1 to v2 of specified weight.
         *Here v1 is any vertex contains user data in the first parameter,
         *and v2 is any vertex contains user data in the second parameter.
         *if there are more than one, then the first will be applied.
         *
         *If the answer is yes then VI will be the pointer
         *pointing to the vertex where the edge from, and EI
         *will be the pointer pointing to this edge.
         *
         *@return false if v1 or v2 are not in graph, or
         *there is no edge from v1 to v2.
         */
       bool IsEdge(VERTEX&, VERTEX&, WEIGHT, VI*, EI*) ;

#ifdef _PGRAPH
       /**
    *For pGraph; Fix the coment
    */
       bool IsEdgeId(VID,int,VI*,EI*);
#endif

       /**Find vertex of _vid in graph.
         *If no such vertex, then v.end() will be returned.
     *@return An iterator pointing to the element found
         */
       VI my_find_VID_eq(const VID _vid);

       /**Find vertex of _vid in graph.
         *If no such vertex, then v.end() will be returned.
     *@return A constant iterator pointing to the element found
         */
       CVI my_find_VID_eqc(const VID _vid) const ;

       /**Find vertex of user data _v in graph.
         *If no such vertex, then v.end() will be returned.
     *@return A constant iterator pointing to the element found
     *@note this method compares every vertex in graph.
         */
       VI my_find_VDATA_eq(const VERTEX& _v) ;

       /**Find vertex of user data _v in graph.
         *If no such vertex, then v.end() will be returned.
     *@return An iterator pointing to the element found
         *@note this method compares every vertex in graph.
         */
       CVI my_find_VDATA_eqc(const VERTEX& _v) const ;

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
  //    Protected: DATA
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    vector< Vertex >    v;  ///< vertices (with adj lists)


    VI begin(){
      return v.begin();
    }
    CVI begin() const{
      return v.begin();
    }
    VI end(){
       return v.end();
    }
    CVI end() const {
      return v.end();
    }

#ifdef DYNAMIC_GRAPH
    //* map VID to LCID
    //* make Vertex search efficient for dynamicly changed graphs 
    //* w/ frequent add and delete of vertices
    //* only used internally in IsVertex function
      map<VID,int> gid_localid_map;
#endif

#ifdef _PGRAPH
    void define_type(stapl::typer &t)
    {
      AbstractGraph<VERTEX>::define_type(t);
      t.local(v);
      //gid_localid_map is not transfered, 
      //but created for sub_graph on each processor
    }

    
#ifdef _TESTBGRAPH    
    void SetV(const BaseGraph<VERTEX,WEIGHT>& _g) {
      v = _g.v;
    }
    void SetV1(const vector<Vertex>& _v1) {
      for(int i=0; i<_v1.size(); ++i)
    v.push_back(*(_v1.begin()+i));
    }
    void PutV() {
        stapl::async_rmi(1,getHandle(),&BaseGraph<VERTEX,WEIGHT>::SetV,*this);
    }
    void PutV1(const vector<Vertex>& _vv) {
        stapl::async_rmi(1,getHandle(),&BaseGraph<VERTEX,WEIGHT>::SetV1,_vv);
    }
#endif   //TESTBGRAPH end

#endif   //_PGRAPH end

#ifdef DYNAMIC_GRAPH
  //===========================
  //bookkeeping for gid_localid_map
  //never called from remote
  //encapsulated into pGraph::_Add/DeleteVertex methods, 
  //which makes remote calls unnecessary
  //===========================
    const map<VID,int>& GetLocalIDMap() const { 
      return gid_localid_map;
    }

    void SetLocalIDMap(const map<VID,int>& _m) { 
      gid_localid_map = _m;
    }

    bool IsInLocalIDMap(VID _gid) {
      if(gid_localid_map.find(_gid) != gid_localid_map.end() )
    return true;
      else 
    return false;
    }

    bool IsInLocalIDMap(VID _gid,  map<VID,int>::iterator& _it) {
      _it= gid_localid_map.find(_gid);
      if(_it != gid_localid_map.end()) 
    return true;
      else 
    return false;
    }

    void Add2LocalIDMap(VID _gid, int _lcid){
      pair<VID,int> mypair(_gid,_lcid);
      gid_localid_map.insert(mypair);
    }

    void RemoveFromLocalIDMap(VID _gid){ 
      //printf("vertex %d erased from LocalIDMap\n",_gid);
      gid_localid_map.erase(_gid);
    }

    void DisplayVIDLocalIDMap(){
      for(map<VID,int>::iterator it=gid_localid_map.begin(); 
      it!=gid_localid_map.end(); ++it){
    cout<<"VID : "<<it->first<<"  with Local ID: "<<it->second<<endl;
      }
    }
#endif


   void SimpleSort()
   {
 
    //Simple sort of the vertices of the pGraph.
    //First sorts the vertices using VID
    //Then sorts using the PID ....Only applicable in the case of the pGraph       
#ifdef _PGRAPH                

        comparebyvid<WtVertexType<VERTEX,WEIGHT> > _c1;
        stable_sort(v.begin(),v.end(),_c1);

        comparebypid<WtVertexType<VERTEX,WEIGHT> > _c2;
        stable_sort(v.begin(),v.end(),_c2);
#endif

  }     


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data members and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
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

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  METHODS FOR template BaseGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////


//==================================
// BaseGraph class Methods: Constructors and Destructor
//==================================

template<class VERTEX,class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>::
BaseGraph(){
}

template<class VERTEX,class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>::
BaseGraph(int _sz) {
    v.reserve(_sz);
}

template<class VERTEX,class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>::
BaseGraph(int _sz, int _edgelistsz)
: AbstractGraph<VERTEX> (_edgelistsz) 
{
    v.reserve(_sz);
}

template<class VERTEX, class WEIGHT> 
BaseGraph<VERTEX,WEIGHT>:: 
~BaseGraph(){
}


//==================================
// BaseGraph class Methods: Adding & Deleting Vertices
//==================================

template<class VERTEX, class WEIGHT> 
VID 
BaseGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v) {
    VID vid = this->vertIDs++;
    Vertex newVertex(_v,vid,this->reserveEdgesPerVertex);
    v.push_back(newVertex);
#ifdef DYNAMIC_GRAPH
    //Add2LocalIDMap(vid,vid);  //not very useful, just to make it consistant.
    Add2LocalIDMap(vid,v.size()-1);  //-gabrielt
#endif
    this->numVerts++;
    return (vid); // return vertex id (not nec. index)
}

template<class VERTEX, class WEIGHT>
VID
BaseGraph<VERTEX,WEIGHT>::
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
BaseGraph<VERTEX,WEIGHT>::
AddVertex(const VERTEX& _v, VID _vid) {
    VID vid = _vid;
    Vertex newVertex(_v,vid,this->reserveEdgesPerVertex);
    v.push_back(newVertex);
#ifdef DYNAMIC_GRAPH
    //Add2LocalIDMap(vid,this->numVerts);  //useful
    Add2LocalIDMap(vid,v.size()-1);  //-gabrielt
#endif
    this->numVerts++;
    return (vid); // return vertex id (not nec. index)
}


template<class VERTEX, class WEIGHT> 
int 
BaseGraph<VERTEX,WEIGHT>::
DeleteVertex(VERTEX& _v1) {
  VI v1;
  if ( IsVertex(_v1,&v1) ) { 
    DeleteAllEdgesToV(v1->vid);
    v.erase(v1);
    this->numVerts--;
#ifdef DYNAMIC_GRAPH
    RemoveFromLocalIDMap(v1->vid);
#endif
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << "\nDeleteVertex: vertex not in graph";
#endif
    return ERROR;
  }
}

template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteVertex(VID _v1id) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
      DeleteAllEdgesToV(_v1id);
      v.erase(v1);
      this->numVerts--;
#ifdef DYNAMIC_GRAPH
      RemoveFromLocalIDMap(_v1id);
#endif
      return OK;
    } else {
#ifndef QUIETGRAPH
      cout << "\nDeleteVertex: vertex not in graph";
#endif
      return ERROR;
    }
}


template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteVertices(vector<VID>& _vv) {

  cout<<"THIS METHOD SHOULD not BE CALLED"<<endl<<endl;
  VI v1;
  vector<VID>::iterator vit;
  
  for(vit = _vv.begin();vit != _vv.end();vit++){
    //for every vertex in input vector
    if(IsVertex(*vit,&v1)){
      //v1 points to the right place
      v1->edgelist.clear();
    v1->predecessors.clear();
    }
  } 
  
  for(VI vi=v.begin();vi!=v.end();vi++){
    if(vi->IsGhost()) continue;
    
    if(vi->edgelist.size() == 0 && vi->predecessors.size()==0) continue;
    EI ei=vi->edgelist.begin();
    while(ei != vi->edgelist.end()){
      vit == find(_vv.begin(),_vv.end(),ei->vertex2id);
    if(vit == _vv.end()) {
      ei++;
      continue;
    }
    ei=vi->edgelist.erase(ei);
    }
  }
  
  //now I have to compact;
  int dest=-1;
  int pos=0;
  for(VI vi=v.begin();vi!=v.end();vi++){
    
    if(vi->edgelist.size() == 0 && vi->predecessors.size()==0) {
      if(dest== -1) dest=pos;
      pos++;
      continue;
    }
    if(dest != -1){
      //here we are if there is a spot were I can move;
      v[dest] = v[pos];
      dest++;
    }
    pos++;
  }
  //and here remove the ghosts of the nodes removed;  
  return OK;
}


#ifdef DYNAMIC_GRAPH
template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
UpdateMap(){
  gid_localid_map.clear();
  int k=0;
  for(VI vi=v.begin();vi!=v.end();vi++){
    gid_localid_map[vi->vid] = k;
    k++;
  }
  return k;
}
#endif




template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
EraseGraph() {
  v.clear();
  this->vertIDs = this->numVerts = this->numEdges = 0;
#ifdef DYNAMIC_GRAPH
  gid_localid_map.clear();
#endif 
  return OK;
}


//==================================
// BaseGraph class Methods: Modifying Vertices
//void PutData(VID, VERTEX);
//void SetPredecessors();
//==================================

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>:: 
GetVertexCount() const {
  return v.size();
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>:: 
GetEdgeCount() const {
    return this->numEdges;
}

template<class VERTEX, class WEIGHT>
VID
BaseGraph<VERTEX,WEIGHT>:: 
GetNextVID() const {
    return this->vertIDs;
}

template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>:: 
PutData(VID _vid, VERTEX _v){ 
    VI v1;
    if ( IsVertex(_vid,&v1) ) {
        v1->data = _v;
    }
}

template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>:: 
SetStartID(VID _startvid) {
    this->vertIDs = _startvid;
}

template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>:: 
SetnumVerts(int _num) {
    this->numVerts = _num;
}
template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>:: 
SetnumEdges(int _num) {
    this->numEdges = _num;
}

template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>:: 
DisplayGraph() const {
    CVI vi;
    int i;
    cout<<endl;
    for (vi = this->v.begin(), i=0; vi < this->v.end(); vi++, i++) {
      cout << setw(3) << i << ": ";
      vi->DisplayEdgelist(1);  //always assume weighted graph
      cout<<endl;
    }
}
//==================================
// BaseGraph class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, EI _ei) {
    VI v1, v2;
    VID v2id = _ei->vertex2id;
    int res=OK;
    WEIGHT weight = _ei->weight;
    
    if (IsVertex(_v1id,&v1) && IsVertex(v2id,&v2) ) {
      res = v1->AddEdge(v2id,weight);
      this->numEdges++;
      return res;
    } else {
#ifndef QUIETGRAPH
      cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
      return ERROR;
    }
}


template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, WEIGHT _weight) {
  VI v1;
  int res=OK;
  if (IsVertex(_v1id,&v1) && IsVertex(_v2id) ) {
    res = v1->AddEdge(_v2id,_weight);
    this->numEdges++;
    return res;
  } else {
#ifndef QUIETGRAPH
    cout << endl << "AddEdge: v1 " << _v1id << " and/or v2 " << _v2id << "not in graph" ;
#endif
    return ERROR;
    }
}


template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1id, VID _v2id, pair<WEIGHT,WEIGHT> _wtpair ) {
  VI v1,v2;
  if (IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) ) {
    v1->AddEdge(_v2id,_wtpair.first);
    v2->AddEdge(_v1id,_wtpair.second);
    this->numEdges += 2;
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
    return ERROR;
  }
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
  int res=OK;
  VI v1, v2;
  if (IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
    res = v1->AddEdge(v2->vid,_weight);
    this->numEdges++;
    return res;
  } else {
#ifndef QUIETGRAPH
    cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
    return ERROR;
  }
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT> _wtpair) {
  VI v1, v2;
  if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
    v1->AddEdge(v2->vid,_wtpair.first);
    v2->AddEdge(v1->vid,_wtpair.second);
    this->numEdges += 2;
    return OK;
  } else {
#ifndef QUIETGRAPH
    cout << "\nAddEdge: vertex 1 and/or vertex 2 not in graph";
#endif
    return ERROR;
  }
}

/////////////////////////////////////////////////////////////////

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, VI* _v1ptr, EI* _e12ptr)  {
    VI v1;
    EI e12;
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
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id)  {
    VI v1;
    EI e12;
    if ( IsVertex(_v1id,&v1) ) {
        if (v1->IsEdge(_v2id,&e12)) {
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
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VID _v1id, VID _v2id, WEIGHT _weight, VI* _v1ptr, EI* _e12ptr)  {
    VI v1;
    EI e12;
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
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, VI* _v1ptr, EI* _e12ptr)  {
    VI v1,v2;
    EI e12;
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
BaseGraph<VERTEX,WEIGHT>:: 
IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, VI* _v1ptr, EI* _e12ptr)  {
    VI v1,v2;
    EI e12;
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


#ifdef _PGRAPH
template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsEdgeId(VID _v1id, int _edgeid, VI* _v1ptr, EI* _e12ptr)  {
    VI v1;
    EI e12;
    if ( IsVertex(_v1id,&v1) ) {
        if (v1->IsEdgeId(_edgeid,&e12)) {
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
//??const version
#endif

//////////////////////////////////////////////////////////////////

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
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
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VID _v1id) {
  VI v1;
  if ( IsVertex(_v1id,&v1) ) {
    this->numEdges -= v1->edgelist.size();
    v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
  }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VID _vid) {
    DeleteAllEdgesToV(_vid);
    DeleteAllEdgesFromV(_vid);
}

// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VID _v1id, VID _v2id, int _n) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
      this->numEdges -= v1->DeleteXEdges(_v2id,_n);
      return OK;
    } else {
      return ERROR;
    }
}

#ifdef _PGRAPH
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteEdgeId(VID _v1id, int _edgeid) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
      return v1->DeleteEdgeId(_edgeid);
    } else {
      return ERROR;
    }
}
#endif

// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _weight, int _n) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
        this->numEdges -= v1->DeleteXEdges(_v2id,_weight,_n);
        return OK;
    } else {
        return ERROR;
    }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
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
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdgesFromV(VERTEX& _v1) {
  VI v1;
  if ( IsVertex(_v1,&v1) ) {
    this->numEdges -= v1->edgelist.size();
    v1->edgelist.erase( v1->edgelist.begin(), v1->edgelist.end() );
  }
}

template<class VERTEX, class WEIGHT>
void 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteAllEdges(VERTEX& _v) {
    DeleteAllEdgesToV(_v);
    DeleteAllEdgesFromV(_v);
}


// default: delete all edges (v1,v2), otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n) {
    VI v1,v2;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
      this->numEdges -= v1->DeleteXEdges(v2->vid,_n);
      return OK;
    } else {
      return ERROR;
    }
}

// default: delete all edges (v1,v2) of specified weight, otherwise delete _n
template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>:: 
DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight, int _n) {
    VI v1,v2;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) ) {
      this->numEdges -= v1->DeleteXEdges(v2->vid,_weight,_n);
      return OK;
    } else {
      return ERROR;
    }
}

//==================================
// BaseGraph class Methods: Finding Vertices & Edges
//==================================

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id) const {
    CVI v1;
    return ( IsVertex(_v1id,&v1) );
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id, CVI*  _v1ptr)  const {
  //??? constant or nonconstant
    CVI v1 = my_find_VID_eqc(_v1id);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;        
    } else {
        return false;
    }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VID _v1id, VI*  _v1ptr) {
  //non constant stuff
    VI v1 = my_find_VID_eq(_v1id);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;        
    } else {
        return false;
    }
}


template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1) const {
    CVI v1;
    return ( IsVertex(_v1,&v1) );
}


template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1, CVI*  _v1ptr) const{
  CVI v1 = my_find_VDATA_eqc(_v1);
  if (v1 != v.end() ) {
    *_v1ptr = v1;
    return true;
  } else {
    return false;
  }
}

template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
IsVertex(VERTEX& _v1, VI*  _v1ptr)  {
  //non constant stuff
    VI v1 = my_find_VDATA_eq(_v1);
    if (v1 != v.end() ) {
        *_v1ptr = v1;
        return true;
    } else {
        return false;
    }
}


//==================================
// BaseGraph class Methods: Getting Data & Statistics
//==================================

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetVerticesVID(vector<VID>& verts) const {
  verts.clear();
    verts.reserve( v.size() );
    for (CVI vi = v.begin(); vi < v.end(); vi++) {
        verts.push_back(vi->vid);
    }
    sort( verts.begin(),verts.end() );
    return verts.size();
}

template<class VERTEX, class WEIGHT>
int 
BaseGraph<VERTEX,WEIGHT>::
GetVerticesVID(vector<VID>& verts, VID _vid, int _n) const {
    verts.clear();
    verts.reserve( _n );
    CVI v1, v2;
    int i;
    if ( IsVertex(_vid,&v1) ) {
        for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
            verts.push_back(v2->vid);
        }
    } else {
#ifndef QUIETGRAPH
        cout << "\nIn GetVerticesVID(VID,int): no vertex VID=" << _vid << " in graph\n";
#endif
    }
    sort( verts.begin(),verts.end() );
    return verts.size();
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetVerticesData(vector<VERTEX>& verts) const {
  verts.clear();
    verts.reserve( v.size() );
    for (CVI vi = v.begin(); vi < v.end(); vi++) {
        verts.push_back(vi->data);
    }
    return verts.size();
}

template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetVerticesData(vector<VERTEX>& verts, VID _vid, int _n) const {
    verts.clear();
    verts.reserve( _n );
    CVI v1, v2;
    int i;
    if ( IsVertex(_vid,&v1) ) {
        for (i = 0, v2 = v1; i < _n && v2 < v.end(); i++, v2++) {
            verts.push_back(v2->data);
        }
    } else {
#ifndef QUIETGRAPH
        cout << "\nIn GetVerticesData(VID,int): no vertex VID=" << _vid << " in graph\n";
#endif
    }
    return verts.size();
}



template<class VERTEX, class WEIGHT>
VERTEX
BaseGraph<VERTEX,WEIGHT>::
GetData(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return v1->data;
    } else {
      //return VERTEX(-1);
      return VERTEX::InvalidData();
    }
}

template<class VERTEX, class WEIGHT>
VERTEX*
BaseGraph<VERTEX,WEIGHT>::
GetReferenceofData(VID _v1id) {
    VI v1;
    if ( IsVertex(_v1id,&v1) ) {
        return (&v1->data);
    } else {
//  VERTEX vv = VERTEX::InvalidData();
        return NULL; 
    }
}

template<class VERTEX, class WEIGHT>
void
BaseGraph<VERTEX,WEIGHT>::
GetAllEdgesToV(VID _v,vector< pair<VID,WEIGHT> > &ToEdges ){
  VI vi2;
  for(vi2=v.begin(); vi2<v.end(); ++vi2){
    for(int i=0; i<vi2->edgelist.size(); ++i){
      if(vi2->edgelist[i].vertex2id==_v){
    pair<VID,WEIGHT> ThisEdge;
    ThisEdge.first=vi2->vid;
    ThisEdge.second=vi2->edgelist[i].weight;
    ToEdges.push_back(ThisEdge);            
      }
    }
  }
}


template<class VERTEX, class WEIGHT>
int
BaseGraph<VERTEX,WEIGHT>::
GetData(vector<VERTEX>& vset, VID _v1id, VID _v2id) const {
    CVI v1, v2;
    vset.clear();
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) ) {
        for (VID i = _v1id; i <= _v2id; i++) { 
            vset.push_back(v1->data);
            v1++;
        }
        return vset.size();
    } else {
        return vset.size(); //in this case return an empty vector
    }
}


template<class VERTEX, class WEIGHT>
VID
BaseGraph<VERTEX,WEIGHT>::
GetVID(VERTEX& _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
        return v1->vid;
    } else {
        return INVALID_VID;
    }
}

  //==================================
  // BaseGraph class Predicates, Comparisons & Operations
  //==================================


template<class VERTEX, class WEIGHT>
/* typename vector<typename BaseGraph<VERTEX,WEIGHT>::Vertex>::iterator */
   //WtVertexType<VERTEX,WEIGHT>*

typename vector<WtVertexType<VERTEX,WEIGHT> >::iterator
BaseGraph<VERTEX,WEIGHT>:: 
my_find_VID_eq(const VID _vid)  {
    VI vi, startvi; 
    
    // find the spot to start looking, hopefully at v[_vid]
    //cout<<"----><"<<v.size()<<" "<<_vid<<endl;
    if(v.size()==0) return v.end();

#ifdef DYNAMIC_GRAPH
    /*     to use the gid_localid_map --ann */
    //fixed by gabrielt to check boundaries
    //cout<<"--- "<<_vid<<endl;
    map<VID,int>::const_iterator mit = gid_localid_map.find(_vid);
    if(mit ==  gid_localid_map.end()) return v.end();
    if ( v.size() > mit->second ) {
      startvi = v.begin() + mit->second;
    } else {
      startvi = v.end() - 1; 
    }

#ifdef DEBUG2
    //cout<<"using local index"<<endl;
#endif

#else
    if ( (signed)v.size() > _vid ) {
        startvi = v.begin() + _vid;
    } else {
        startvi = v.end() - 1; 
    }
#endif
    // look back from v[_vid]
    vi = startvi;
    while ( vi >= v.begin()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi--;
        }
    }
    
    // look forward from v[_vid]
    vi = startvi;
    while ( vi < v.end()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi++;
        }
    }
    
    // if didn't find it return v.end() like STL find
    return v.end();

}

template<class VERTEX, class WEIGHT>
/* typename vector<typename BaseGraph<VERTEX,WEIGHT>::Vertex>::const_iterator */
  //WtVertexType<VERTEX,WEIGHT>*
typename vector<WtVertexType<VERTEX,WEIGHT> >::const_iterator
BaseGraph<VERTEX,WEIGHT>:: 
my_find_VID_eqc(const VID _vid) const {
    CVI vi, startvi; 
    
    // find the spot to start looking, hopefully at v[_vid]
    //cout<<"----><"<<v.size()<<" "<<_vid<<endl;
    if(v.size()==0) return v.end();

#ifdef DYNAMIC_GRAPH
    //fixed by gabrielt to check boundaries
    
    map<VID,int>::const_iterator mit = gid_localid_map.find(_vid);
    //cout<<"--- "<<_vid<<" "<<mit->second<<" "<<v.size()<<endl;
    if(mit ==  gid_localid_map.end()) return v.end();
    //startvi = v.begin() + mit->second;
    if ( v.size() > mit->second ) {
        startvi = v.begin() + mit->second;
    } else {
        startvi = v.end() - 1; 
    }
#ifdef DEBUG
    //cout<<"using local index"<<endl;
#endif
#else
    if ( (signed)v.size() > _vid ) {
        startvi = v.begin() + _vid;
    } else {
        startvi = v.end() - 1; 
    }
#endif
    
    // look back from v[_vid]
    vi = startvi;
    while ( vi >= v.begin()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi--;
        }
    }
    
    // look forward from v[_vid]
    vi = startvi;
    while ( vi < v.end()  ) {
        if ( vi->vid == _vid) {
            return ( vi );
        } else {
            vi++;
        }
    }
    
    // if didn't find it return v.end() like STL find
    return v.end() ;
}

template<class VERTEX, class WEIGHT>
/* typename vector<typename BaseGraph<VERTEX,WEIGHT>::Vertex >::iterator */
typename vector<WtVertexType<VERTEX,WEIGHT> >::iterator
BaseGraph<VERTEX,WEIGHT>::
my_find_VDATA_eq(const VERTEX& _v)  {
    VI vi = v.begin();
    bool found = false;
    while (vi != v.end() && !found) {
      if ( vi->data == _v) {
    found = true;
      } else {
    vi++;
      }
    }
    return (vi);
}

template<class VERTEX, class WEIGHT>
/* typename vector<typename BaseGraph<VERTEX,WEIGHT>::Vertex>::const_iterator */
typename vector<WtVertexType<VERTEX,WEIGHT> >::const_iterator
BaseGraph<VERTEX,WEIGHT>::
my_find_VDATA_eqc(const VERTEX& _v) const  {
    CVI vi = v.begin();
    bool found = false;
    while (vi != v.end() && !found) {
      if ( vi->data == _v) {
    found = true;
      } else {
    vi++;
      }
    }
    return (vi);
}


template<class VERTEX, class WEIGHT>
bool 
BaseGraph<VERTEX,WEIGHT>:: 
#ifdef __HP_aCC
VID_Compare (const Vertex _v1, const Vertex _v2){
#else 
VID_Compare (const Vertex& _v1, const Vertex& _v2){
#endif
    return (_v1.vid < _v2.vid ) ; 
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
WtVertexType(const VERTEX& _data, VID _id){
    data = _data;
    vid = _id;

#ifdef _PGRAPH
    edgeid_cnt=0;
#endif
}

template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
WtVertexType(const VERTEX& _data, VID _id, int _edgelistsz){
    data = _data;
    vid = _id;
    edgelist.reserve( _edgelistsz );
#ifdef _PGRAPH
    edgeid_cnt=0;
#endif
}


template<class VERTEX, class WEIGHT>
WtVertexType<VERTEX,WEIGHT>:: 
~WtVertexType(){
}

//==================================
// Vertex class Methods: Adding & Deleting Edges
//==================================

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdge(VID _v2id, WEIGHT _weight) {
  int res=OK;
  WtEdge newEdge(_v2id, _weight);
#ifdef _PGRAPH
  res = this->GetNextEdgeId();
  newEdge.SetEdgeId(res);
#endif
  edgelist.push_back(newEdge);
  return res;
}

template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>:: 
AddEdgewCheck(VID _v2id, WEIGHT _weight) {
  int res = OK;
  bool found;
  for(EI ei=edgelist.begin(); ei!=edgelist.end(); ei++) {
    if(_v2id == ei->vertex2id) found=true;
  }
  if(!found) {
     WtEdge newEdge(_v2id, _weight);
#ifdef _PGRAPH
     res = this->GetNextEdgeId();
     newEdge.SetEdgeId(res);
#endif
     edgelist.push_back(newEdge);
  }
  return res;
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


#ifdef _PGRAPH
template<class VERTEX, class WEIGHT>
int 
WtVertexType<VERTEX,WEIGHT>::
DeleteEdgeId(int _edgeid) {

  EI ei;
  if (edgelist.size() == 0) return OK;

  if(_edgeid > edgelist.size()){
    ei = edgelist.end();
    ei--;
  }
  else{
    ei = edgelist.begin() + _edgeid;//starting point is related with edgeid
  }

  while ( ei != edgelist.begin() ){
    if(ei->GetEdgeId() == _edgeid){
      edgelist.erase(ei);
      return OK;
    }
    ei--;
  }
  //here we are on the first element
  if(ei->GetEdgeId() == _edgeid){
    edgelist.erase(ei);
    return OK;
  }
  cout<<"BaseGraph.h::DeleteEdgeId Inexistent edge"<<endl;
  return ERROR;
}
#endif

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
IsEdge(VID _v2id, CEI* _ei) const {
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
IsEdge(VID _v2id, EI* _ei)  {
    EI ei = my_find_EID1_eq(_v2id);
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
IsEdge(VID _v2id, WEIGHT _weight, CEI* _ei) const {
    CEI ei = my_find_EIDWT_eqc( pair<VID,WEIGHT>(_v2id,_weight) );
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
IsEdge(VID _v2id, WEIGHT _weight, EI* _ei)  {
    EI ei = my_find_EIDWT_eq( pair<VID,WEIGHT>(_v2id,_weight) );
    if (ei != edgelist.end() ) {
        *_ei = ei;
        return true;
    } else {
        return false;
    }
}


#ifdef _PGRAPH
template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdgeId(int _edgeid, EI* _ei)  {
  EI ei;

  if(edgelist.size() == 0) return false;

  if(_edgeid > edgelist.size()){
    ei = edgelist.end();
    ei--;
  }
  else{
    ei = edgelist.begin() + _edgeid;//starting point is related with edgeid
  }
  
  while ( ei != edgelist.begin() ){
    if(ei->GetEdgeId() == _edgeid){
      *_ei = ei;
      return true;
    }
    ei--;
  }

  if(ei->GetEdgeId() == _edgeid){
    *_ei = ei;
    return true;
  }
  cout<<"BaseGraph.h::IsEdgeID Inexistent edge"<<endl;
  return false;
}

template<class VERTEX, class WEIGHT>
bool 
WtVertexType<VERTEX,WEIGHT>::
IsEdgeId(int _edgeid, CEI* _ei) const {
  CEI ei;

  if(edgelist.size() == 0) return false;

  if(_edgeid > edgelist.size()){
    ei = edgelist.end();
    ei--;
  }
  else{
    ei = edgelist.begin() + _edgeid;//starting point is related with edgeid
  }
  
  while ( ei != edgelist.begin() ){
    if(ei->GetEdgeId() == _edgeid){
      *_ei = ei;
      return true;
    }
    ei--;
  }

  if(ei->GetEdgeId() == _edgeid){
    *_ei = ei;
    return true;
  }
  cout<<"BaseGraph.h::IsEdgeID Inexistent edge"<<endl;
  return false;
}

#endif

//==================================
// Vertex class Methods: Getting Data & Statistics 
//==================================

template<class VERTEX, class WEIGHT>
VERTEX& 
WtVertexType<VERTEX,WEIGHT>::
GetData() {
    return data;
}

template<class VERTEX, class WEIGHT>
const VERTEX& 
WtVertexType<VERTEX,WEIGHT>::
GetData() const {
    return data;
}

template<class VERTEX, class WEIGHT>
VERTEX& 
WtVertexType<VERTEX,WEIGHT>::
GetUserData() {
    return data;

}

template<class VERTEX, class WEIGHT>
const VERTEX& 
WtVertexType<VERTEX,WEIGHT>::
GetUserData() const {
    return data;
}

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>::
SetUserData(const VERTEX& _data) {
    data = _data;
}

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>::
SetUserData(VERTEX& _data) {
    data = _data;
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
    CEI ei;
    if (IsEdge(_v2id,&ei)) {
        return ei->weight;
    } else {
#ifndef QUIETGRAPH
        cout << "\nGetEdgeWeight: edge not in graph";
#endif
        //return WEIGHT::InvalidWeight();
    //???? this has to be fixed
    return WEIGHT(-1);
    }
}


#ifdef _PGRAPH
template<class VERTEX, class WEIGHT>
WEIGHT 
WtVertexType<VERTEX,WEIGHT>::
GetEdgeIdWeight(int edgeid) const {
    CEI ei;
    if (IsEdgeId(edgeid,&ei)) {
        return ei->weight;
    } else {
#ifndef QUIETGRAPH
      cout << "\nGetEdgeWeight: edge not in graph";
#endif
      //return WEIGHT::InvalidWeight();
      //???? this has to be fixed
      return WEIGHT(-1);
    }
}

template<class VERTEX, class WEIGHT>
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
WtVertexType<VERTEX,WEIGHT>::
find_edgeid_eq(const EID _edgeid) const {
  CEI ei;

  if(edgelist.size() == 0) return edgelist.end();

  if(_edgeid > edgelist.size()){
    ei = edgelist.end();
    ei--;
  }
  else{
    ei = edgelist.begin() + _edgeid;//starting point is related with edgeid
  }
  
  while ( ei != edgelist.begin() ){
    if(ei->GetEdgeId() == _edgeid){
      return ei;
    }
    ei--;
  }
  if(ei->GetEdgeId() == _edgeid){
    return ei;
  }
}

template<class VERTEX, class WEIGHT>
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
find_edgeid_eq(const EID _edgeid) {
  EI ei;

  if(edgelist.size() == 0) return edgelist.end();

  if(_edgeid > edgelist.size()){
    ei = edgelist.end();
    ei--;
  }
  else{
    ei = edgelist.begin() + _edgeid;//starting point is related with edgeid
  }
  
  while ( ei != edgelist.begin() ){
    if(ei->GetEdgeId() == _edgeid){
      return ei;
    }
    ei--;
  }
  if(ei->GetEdgeId() == _edgeid){
    return ei;
  }
}


#endif
//==================================
// Vertex class Methods: Display, Input, Output 
//==================================

template<class VERTEX, class WEIGHT>
void 
WtVertexType<VERTEX,WEIGHT>::
DisplayEdgelist(int wt) const {
    cout << "id =" << setw(3) << vid << ", data = ["; 
    cout << data;
    cout << "], edges={";
    {for (CEI ei = edgelist.begin(); ei < edgelist.end(); ei++) {
        ei->DisplayEdge(wt);
        if (ei != edgelist.end() - 1) cout << ", ";
    }}
    cout << "} Predecessors={";

    {for (CEI ei = predecessors.begin(); ei < predecessors.end(); ei++) {
        ei->DisplayEdge(wt);
        if (ei != predecessors.end() - 1) cout << ", ";
    }}
    cout << "} \n";
}

template<class VERTEX, class WEIGHT>
void
WtVertexType<VERTEX,WEIGHT>::
WriteEdgelist(ostream& _myostream,int wt) const {
    
    _myostream << vid << " "; 
    _myostream << data << " "; 
    _myostream << edgelist.size() << " "; 
    
    for (CEI ei = edgelist.begin(); ei != edgelist.end(); ei++) { 
        ei->WriteEdge(_myostream,wt);
        _myostream << " "; 
    }
}

//==================================
// Vertex class Predicate Utilities
//==================================

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
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
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EID1_eq(const EID _eid) {
    EI ei = edgelist.begin();
    bool found = false;
    while (ei != edgelist.end() && !found) {
        if ( ei->vertex2id == _eid) {
            found = true;
        } else {
            ei++;
        }
    }
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EID2_eqc(CEI _start, 
                CEI _end,
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
    return (ei);
}

template<class VERTEX, class WEIGHT>
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EID2_eq(EI _start, 
                EI _end,
                const EID _eid) {
    EI ei = _start;
    bool found = false;
    while (ei != _end && !found) {
        if ( ei->vertex2id == _eid) {
            found = true;
        } else {
            ei++;
        }
    }
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
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
    return (ei);
}

template<class VERTEX, class WEIGHT>
  //WtEdgeType<VERTEX,WEIGHT>*
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::const_iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EIDWT_eqc(const pair<VID,WEIGHT> _wtpair) const  {
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
   return (ei);
}
template<class VERTEX, class WEIGHT>
typename  vector<WtEdgeType<VERTEX,WEIGHT> >::iterator
WtVertexType<VERTEX,WEIGHT>::
my_find_EIDWT_eq(const pair<VID,WEIGHT> _wtpair)  {
   EI ei = edgelist.begin();
   // EI ei = const_cast<EI> (cei);
   
   bool found = false;
   while (ei != edgelist.end() && !found) {
      if ( ei->vertex2id==_wtpair.first && ei->weight == _wtpair.second ) {
         found = true;
      } else {
         ei++;
      }
   }
   return (ei);
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

#ifdef _PGRAPH
template<class VERTEX, class WEIGHT>
WtEdgeType<VERTEX,WEIGHT>:: 
WtEdgeType(VID _v2id, WEIGHT _weight,int _eid){
    vertex2id = _v2id;
    weight = _weight;
    edgeid = _eid;
}
#endif

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
DisplayEdge(int wt) const {
  cout << vertex2id;
  if (wt==1) cout << "(" << weight << ")";
} 

template<class VERTEX, class WEIGHT>
void
WtEdgeType<VERTEX,WEIGHT>::
WriteEdge(ostream& _myostream,int wt) const {
  _myostream << vertex2id<<" ";
  if(wt == 1) _myostream << weight << " ";
}

};

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


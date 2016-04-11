//this will be the new graph

/*******************
TO DO:
- clean the code and do some renamings
- see why crashes GetCC statistics(crashes for directed !!! Normal)
*/


#ifndef Graph_h
#define Graph_h
#include "BaseGraph.h"
#ifdef _PGRAPH
#include <runtime.h>
#include "rmitools.h"
#endif

#include <string.h>

namespace graph{

////////////////////////////////////////////////////////
//
// undirected / directed graph
///////////////////////////////////////////////////////
/**Undirected Graph class. Here we declare methods that has to handle the fact that the graph
 *is undirected. for example AddEdge, DeleteEdge are different from similar methods for Directed
 *Graph.All these methods are inheritted by Graph when the class is used. 
*/
template<class VERTEX, class WEIGHT=int, class Base=BaseGraph<VERTEX, WEIGHT> >
class UG: public  virtual Base{
  public:

  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

/*   void define_type(stapl::typer &t) { */
/*     BaseGraph<VERTEX,WEIGHT>::define_type(t); */
/*   } */

  typedef typename Base::VI   VI;   ///<VI Vertex Iterator
  typedef typename Base::CVI  CVI;  ///<CVI Constant Vertex Iterator
  typedef typename Base::RVI  RVI;  ///<RVI Reverse Vertex Iterator
  typedef typename Base::CRVI CRVI; ///<CRVI Constant Reverse Vertex Iterator

  typedef typename Base::EI   EI;   ///<EI Edge Iterator
  typedef typename Base::CEI  CEI;  ///<CEI Constant Edge Iterator
  typedef typename Base::REI  REI;  ///<REI Reverse Edge Iterator
  typedef typename Base::CREI CREI; ///<CREI Constant Reverse Edge Iterator

  

  inline UG(){
  }
  inline UG(int _sz): Base (_sz){
  }
  inline UG(int _sz,int _edgelistsz) : Base (_sz,_edgelistsz) {
  }
  ~UG(){}
  inline int check_directed() const {
    return 0;
  }

  /**@name Adding & Deleting Edges*/
  //@{
  /**Add edge v1->v2, and v2->v1 to graph with same weight.
   *Here v1 is any vertex contains user data in the first parameter,
   *and v2 is any vertex contains user data in the second parameter.
   *if there are more than one, then the first will be applied.
   *@return ERROR if v1 and/or v2 are not in graph.
   *@return ERROR if v1 and v2 have been connected.
   *@note enven 2 edges are created, but they are counted as one edge.
   */
  inline int AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    if(Base::AddEdge(_v1,_v2,_weight) == ERROR) return ERROR;
    return Base::AddEdge(_v2,_v1,_weight);
  }
 
  /**Add edge vid1->vid2, and vid2->vid1 to graph with same weight.
   *@return ERROR if vid1 and/or vid2 are not in graph.
   *@return ERROR if vid1 and vid2 have been connected.
   *@note enven 2 edges are created, but they are counted as one edge.
   */
  inline int AddEdge(VID _v1, VID _v2, WEIGHT _weight) {
    if(Base::AddEdge(_v1,_v2,_weight)==ERROR) return ERROR;
    return Base::AddEdge(_v2,_v1,_weight);
  }

  /**Add edge vid1->vid2, and vid2->vid1 to graph with 2 different weights.
   *@return ERROR if vid1 and/or vid2 are not in graph.
   *@return ERROR if vid1 and vid2 have been connected.
   *@note enven 2 edges are created, but they are counted as one edge.
   */
  int  AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _p){
    if(Base::AddEdge(_v1,_v2,_p.first)==ERROR) return ERROR;
    return Base::AddEdge(_v2,_v1,_p.second);
  }
  /**Add edge v1->v2, and v2->v1 to graph with 2 different weights.
   *Here v1 is any vertex contains user data in the first parameter,
   *and v2 is any vertex contains user data in the second parameter.
   *if there are more than one, then the first will be applied.
   *
   *@return ERROR if v1 and/or v2 are not in graph.
   *@return ERROR if v1 and v2 have been connected.
   *@note enven 2 edges are created, but they are counted as one edge.
   */
  int  AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _p){
    if(Base::AddEdge(_v1,_v2,_p.first)==ERROR) return ERROR;
    return Base::AddEdge(_v2,_v1,_p.second);
  }
  /**Delete all edges from vid1 to vid2 and from vid2 to vid1.
   *No matter what n is given, it always chage n to -1.
   *
   *@param n number of edges will be delete.
   *@note vid1 an vid2 should be in this graph
   *@note enven 2 edges are deleted, but they are counted as one edge.
   *@return ERROR if vid1 and/or vid2 are not found. OK if ok.
   */
  int DeleteEdge(VID _v1id, VID _v2id, int _n=-1) {
    VI v1, v2;
    VI vi1;EI ei;
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) && this->IsEdge(_v1id,_v2id,&vi1,&ei) ) {
      int ok1 = v1->DeleteXEdges(_v2id,-1);
      int ok2 = v2->DeleteXEdges(_v1id,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1 
    this->numEdges--;
    return OK;
      } 
    }
    return ERROR;
  }

  /**Delete n edges from vid1 to vid2 and from vid2 to vid1 
   *of specified weight.
   *
   *No matter what n is given, it always chage n to -1.
   *
   *@param WEIGHT the edges of this weught will be deleted
   *@note vid1 an vid2 should be in this graph
   *@note enven 2 edges are deleted, but they are counted as one edge.
   *@return ERROR if vid1 and/or vid2 are not found. OK if ok.
   */
  int DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _w, int _n=-1) {
    VI v1, v2;
    EI ei;VI cv1;
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) &&this->IsEdge(_v1id,_v2id,_w,&cv1,&ei) ) {
      int ok1 = v1->DeleteXEdges(_v2id,-1);
      int ok2 = v2->DeleteXEdges(_v1id,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1 
    this->numEdges--;
    return OK;
      } 
    }
    return ERROR;
  }

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
   *@return ERROR if v1 and/or v2 are not found. OK if ok.
   */  
  int DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n=-1) {
    VI v1, v2;
    VI vi1;
    EI ei;

    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) && this->IsEdge(_v1,_v2,&vi1,&ei) ) {
      int ok1 = v1->DeleteXEdges(v2->vid,-1);
      int ok2 = v2->DeleteXEdges(v1->vid,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1
    this->numEdges--; 
    return OK;
      }
    }
    return ERROR;
  }
  
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
   *@return ERROR if v1 and/or v2 are not found. OK if ok.
   */
  int DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w, int _n=-1) {
    VI v1, v2;
    VI cv1;
    EI ei;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) && this->IsEdge(_v1,_v2,_w,&cv1,&ei) ) {
      int ok1 = v1->DeleteXEdges(v2->vid,-1);
      int ok2 = v2->DeleteXEdges(v1->vid,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1
    this->numEdges--; 
    return OK;
      }
    }
    return ERROR;
  }
  //@}

  /**@name Accesing the neighbors(vertices/edges)*/
  //@{
  /**Get Degree of this specified Vertex.
   *@note For undirected graph, outgoing degree is the same as incoming degree.
   *@return Degree if specified VID is found in graph. Otherwise ERROR will be returned.
   */
  int GetVertexOutDegree(VID _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
      return v1->edgelist.size();
    } else {
      cout << "\nGetVertexDegree: vertex "<< v1->vid << " not in graph";
      return ERROR;
    }
  }
  
  /**Get vertices which are adjacent to this specified vertex.
   *@param _v1id the vertex for which we want adjacent information
   *@param _succ vector of VIDs which are the VIDs of those who are next to the specified vertex.
   *@return the number of elements pushed in _succ
   *If this specified VID is not found, an empty list will be returned and error message
   *will be in standard output.
   *@see GetSuccessors
   */
  int GetAdjacentVertices(VID _v1id, vector<VID>& _succ) const {
    CVI v1;    
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    _succ.push_back(ei->vertex2id);
      }
    } else {
      cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }
  
  /**Get vertices which are adjacent to this specified vertex(user data).
   *@param _v1id the vertex for which we want adjacent information
   *@param _succ vector of VIDs which are the VIDs of those who are next to the specified vertex.
   *@return the number of elements pushed in _succ
   *If this specified VERTEX is not found, an empty list will be returned and error message
   *will be in standard output.
   *@see GetSuccessors
   */
  int GetAdjacentVertices(VERTEX& _v1, vector<VID>& _succ) const {
    return GetAdjacentVertices( GetVID(_v1), _succ);
  }
  /** Used internally by  Dikstra; It has the same functionality as the above 
      two functions.
   */
  int GetDijkstraInfo(VID _v1id, vector<VID>& _succ) const {
    return GetAdjacentVertices(_v1id,_succ);
  }

  /**Get data for vertices which are adjacent to this specified vertex(user data).
   *@param _v1id the vertex for which we want adjacent information
   *@param _succ vector of user data which is the data of those who are next to the specified vertex.
   *@return the number of elements pushed in _succ
   *If the specified VID is not found, an empty list will be returned and error message
   *will be in standard output.
   *@see GetSuccessors
   */
  int GetAdjacentVerticesDATA(VID _v1id, vector<VERTEX>& _succ) const {
    CVI v1,v2;
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    if ( IsVertex(ei->vertex2id,&v2) )
      _succ.push_back(v2->data);
      }
    } else {
      cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }
  /**Get data for vertices which are adjacent to this specified vertex(user data).
   *@param _v1 the vertex for which we want adjacent information
   *@param _succ vector of user data which is the data of those who are next to the specified vertex.
   *@return the number of elements pushed in _succ
   *If the specified VERTEX is not found, an empty list will be returned and error message
   *will be in standard output.
   *@see GetSuccessors
   */
  int GetAdjacentVerticesDATA(VERTEX& _v1, vector<VERTEX>& _succ) const {
    return GetAdjacentVerticesDATA( GetVID(_v1), _succ );
  }

  /**Get Edges which is incident to this specified vertex.
   *Incident edges are edges around this specified vertex.
   *@return a list of edges which is defined by 2 VIDs of its endpoints.
   *Empty list will be returned if this specified VID 
   *is not in graph and error message will be in standard output.
   */
  int GetIncidentEdges(VID _v1id,vector< pair<VID,VID> >& iedges) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    pair<VID,VID> nextedge(_v1id,ei->vertex2id);
    iedges.push_back( nextedge );
      }
    } else {
      cout << "\nGetIncidentEdges: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }
  /**Get Edges which is incident to this specified vertex and user data associated with
   *these edges.
   *Incident edges are edges around this specified vertex.
   *@return a list of edges which is defined by 2 user data of its endpoints.
   *Empty list will be returned if this specified VID 
   *is not in graph and error message will be in standard output.
   */
  /*
  int GetIncidentEdgesVData(VID _v1id, vector< pair<VERTEX,VERTEX> >& iedges) const{
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    pair<VERTEX,VERTEX> nextedge( this->GetData(_v1id), this->GetData(ei->vertex2id));
    iedges.push_back( nextedge );
      }
    } else {
      cout << "\nGetIncidentEdgesVData: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }
  */

  /**Get Edges which is incident to this specified vertex.
   *Incident edges are edges around this specified vertex.
   *@return a list of edges which is defined by 2 VIDs of its endpoints and the edge weight.
   *Empty list will be returned if this specified VID 
   *is not in graph and error message will be in standard output.
   */
  int GetIncidentEdges(VID _v1id,vector< pair<pair<VID,VID>,WEIGHT> >& iedges) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    pair<VID,VID> nextedge(_v1id,ei->vertex2id);
    pair<pair<VID,VID>,WEIGHT> nextedgewt(nextedge,ei->weight);
    iedges.push_back( nextedgewt );
      }
    } else {
      cout << "\nGetIncidentEdges: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }
  
  /**Get Edges which is incident to this specified vertex and user data associated with
   *these edges.
   *Incident edges are edges around this specified vertex.
   *@return a list of edges which is defined by 2 user data of its endpoints and
   *the weight of edge. Empty list will be returned if this specified VID 
   *is not in graph and error message will be in standard output.
   */
  int GetIncidentEdgesVData(VID _v1id,vector< pair<pair<VERTEX,VERTEX>,WEIGHT> >& iedges) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    pair<VERTEX,VERTEX> nextedge( this->GetData(_v1id), this->GetData(ei->vertex2id));
    pair<pair<VERTEX,VERTEX>,WEIGHT> nextedgewt(nextedge,ei->weight);
    iedges.push_back( nextedgewt );
      }
    } else {
      cout << "\nGetIncidentEdgesVData: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }
  //@}
};

/**This class encapsulates the methods related with the fact the graph is directed. 
 *We have methods to handle the successors and predecessors.All these methods are inheritted by 
 *Graph when the class is used.
*/
template<class VERTEX, class WEIGHT=int, class Base=BaseGraph<VERTEX, WEIGHT> >
class DG: public  virtual Base{
  public:

  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

/*   void define_type(stapl::typer &t) { */
/*     Base::define_type(t); */
/*   } */

  typedef typename Base::VI   VI;   ///<VI Vertex Iterator
  typedef typename Base::CVI  CVI;  ///<CVI Constant Vertex Iterator
  typedef typename Base::RVI  RVI;  ///<RVI Reverse Vertex Iterator
  typedef typename Base::CRVI CRVI; ///<CRVI Constant Reverse Vertex Iterator

  typedef typename Base::EI   EI;   ///<EI Edge Iterator
  typedef typename Base::CEI  CEI;  ///<CEI Constant Edge Iterator
  typedef typename Base::REI  REI;  ///<REI Reverse Edge Iterator
  typedef typename Base::CREI CREI; ///<CREI Constant Reverse Edge Iterator

  inline DG(){
    //setDirectness(1);
  }
  inline DG(int _sz): Base (_sz){
    //setDirectness(0);
  }
  inline DG(int _sz,int _edgelistsz) : Base (_sz,_edgelistsz) {
    //setDirectness(0);
  }
  ~DG(){}
  inline int check_directed() const {
    return 1;  
  }


  //=======================================================
  //The following methods need to call SetPredecessors() first
  //to initialize predecessor vector
  //=======================================================

  /**@name Accesing predecessors/succesors(vertices/edges)*/
  //@{
  /**Initialize predecessors in the data field of Vertex.
   *Predecessors tells client where the edges that 
   *connected to this vertex are from. Return false if the predecessors 
   *already set.
   */
  void SetPredecessors() {
    VI v1, v2;
    VID _v2id;
    bool DoneSet = false;
    
    //check if SetPredecessors() already called
    for(v1 = this->v.begin(); v1 < this->v.end(); v1++) {
      if(!v1->predecessors.empty()) {
    DoneSet = true;
    cout<<"\nSetPredecessors() already called."<<endl;
    return;
    }
    }
    
    for(v1 = this->v.begin(); v1 < this->v.end(); v1++) {
      for (EI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    _v2id = ei->vertex2id;
    if ( IsVertex(_v2id, &v2) ) {
      WtEdge newEdge( v1->vid, ei->weight );
      v2->predecessors.push_back(newEdge);
    }
      }
    }
  }

  void SetPredecessorsForce() {
    VI v1, v2;
    VID _v2id;

    for(v1 = this->v.begin(); v1 < this->v.end(); v1++) {
      v1->predecessors.clear();
    }

    for(v1 = this->v.begin(); v1 < this->v.end(); v1++) {
      for (EI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    _v2id = ei->vertex2id;
    if ( IsVertex(_v2id, &v2) ) {
      WtEdge newEdge( v1->vid, ei->weight );
      v2->predecessors.push_back(newEdge);
    }
      }
    }
  }

  /**Get out degree of a given vertex.
   *out degree of a vertex is number of edges
   *that are from this vertex to any other vertex in graph.
   *@return ERROR if no this vertex.
   */
  int GetVertexOutDegree(VID _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
      return v1->edgelist.size();
    } else {
      return ERROR;
    }
  }
  /**Get VIDs which are connected to by edges going out from
   *specified VID.
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   */  
  int GetSuccessors(VID _v1id, vector<VID>& _succ) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    _succ.push_back(ei->vertex2id);
      }
    } else {
        cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }
  /**Get VIDs which are connected to by edges going out from
   *specified VID.
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   */  
  int GetSuccessors(VI _vi, vector<VID>& _succ) const {
    _succ.clear();
    _succ.reserve( _vi->edgelist.size() );
    for (CEI ei = _vi->edgelist.begin(); ei != _vi->edgelist.end(); ei++) {
      _succ.push_back(ei->vertex2id);
    }
    return _succ.size();
  }


  /**Get VERTEXs which are connected to by edges going out from
   *specified VID.
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   */
  int GetSuccessorsDATA(VID _v1id, vector<VERTEX>& _succ) const {
    CVI v1,v2;
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    if ( IsVertex(ei->vertex2id,&v2) )
      _succ.push_back(v2->data);
      }
    } else {
      cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }

  /**Get VIDs which are connected to by edges going out from
   *specified v.
   *Here v is any vertex contains user data in the parameter
   *if there are more than one, then the first will be applied.
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   */
  int GetSuccessors(VERTEX& _v1, vector<VID>& _succ) const {
    return GetSuccessors( GetVID(_v1), _succ );
  }

  /**Get VERTEXs which are connected to by edges going out from
   *specified _v1.
   *Here _v1 is any vertex contains user data in the parameter
   *if there are more than one, then the first will be applied.
   *return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   *@return the number of elements pushed in _succ
   */
  int GetSuccessorsDATA(VERTEX& _v1, vector<VERTEX>& _succ) const {
    return GetSuccessorsDATA( GetVID(_v1), _succ );
  }
  /** internal used by Dijkstra SSP
      Same functionality as the above functions
      */
  int GetDijkstraInfo(VID _v1id, vector<VID>& _succ) const {
    return GetSuccessors(_v1id,_succ);
  }
  /**Get all predecessors of a specified VID as a vector<VID>.
   *@note Need to call SetPredecessors() first to initialize predecessor vector
   *before using this method.
   *
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   *@see SetPredecessors
   */
  int GetPredecessors(VID _v1id, vector<VID>& _pred) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      _pred.clear();
      _pred.reserve( v1->predecessors.size() );
      for (CEI ei = v1->predecessors.begin(); ei != v1->predecessors.end(); ei++) {
    _pred.push_back(ei->vertex2id);
      }
    } else {
      cout << "\nGetPredecessors: vertex "<< _v1id << " not in graph";
    }
    return _pred.size();
  }

  /**Get all predecessors of a specified VID as a vector<VID>.
   *@note Need to call SetPredecessors() first to initialize predecessor vector
   *before using this method.
   *
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   *@see SetPredecessors
   */
  int GetPredecessors(VI _vi, vector<VID>& _pred) const {
    _pred.clear();
    _pred.reserve( _vi->predecessors.size() );
    for (CEI ei = _vi->predecessors.begin(); ei != _vi->predecessors.end(); ei++) {
      _pred.push_back(ei->vertex2id);
    }
    return _pred.size();
  }



  /**Get all predecessors of a specified VID as a vector<VERTEX>.
   *@note Need to call SetPredecessors() first to initialize predecessor vector
   *before using this method.
   *
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   *@see SetPredecessors
   */
  int GetPredecessorsDATA(VID _v1id,vector<VERTEX>& _pred) const {
    CVI v1,v2;
    if ( IsVertex(_v1id,&v1) ) {
      _pred.clear();
      _pred.reserve( v1->predecessors.size() );
      for (CEI ei = v1->predecessors.begin(); ei != v1->predecessors.end(); ei++) {
        if ( IsVertex(ei->vertex2id,&v2) )  
          _pred.push_back(v2->data);
      }
    } else {
      cout << "\nGetPredecessors: vertex "<< _v1id << " not in graph";
    }
    return _pred.size();
  }

  /**Get all predecessors of a specified v as a vector<VID>.
   *Here v is any vertex contains user data in the parameter
   *if there are more than one, then the first will be applied.
   *
   *@note Need to call SetPredecessors() first to initialize predecessor vector
   *before using this method.
   *
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   */
  int GetPredecessors(VERTEX& _v1,vector<VID>& _pred) const {
    return GetPredecessors( GetVID(_v1), _pred );
  }

  /**Get all predecessors of a specified v as a vector<VERTEX>.
   *Here v is any vertex contains user data in the parameter
   *if there are more than one, then the first will be applied.
   *
   *@note Need to call SetPredecessors() first to initialize predecessor vector
   *before using this method.
   *
   *@return empty vector If there is no such vertex found and
   *error message will be output to standard output.
   */
  int GetPredecessorsDATA(VERTEX& _v1,vector<VERTEX>& _pred) const {
    return GetPredecessorsDATA( GetVID(_v1), _pred );
  }

  /**Return VIDs of vertices which are sources.
   *Source vertex is the vertex that has no incoming edges (i.e. no predecessors)
   *This method returns all sources in the graph.
   *Need to call SetPredecessors() for the graph before using this method
   */
  int GetSources(vector<VID>& sourcevids) const {
    CVI v1;
    sourcevids.clear();
    for(v1=this->v.begin(); v1!=this->v.end(); v1++) {
      if(v1->predecessors.empty()) sourcevids.push_back(v1->vid);
    }
    return sourcevids.size();
  }

  /**Return VIDs of vertices which are sinks.
   *Sink vertex is the vertex that has no outgoing edges (i.e. no successors)
   *This method returns all sinks in the graph.
   */
  int GetSinks(vector<VID>& sinkvids) const {
    CVI cv1;
    VI v1;
    sinkvids.clear();
    for(cv1=this->v.begin(); cv1!=this->v.end(); cv1++) {
      //        v1 = const_cast<VI> (cv1);
      if(cv1->edgelist.empty()) sinkvids.push_back(cv1->vid);
    }
    return sinkvids.size();//CHANGE
  }
  //@}
};


////////////////////////////////////////////////////////
//
//  weighted/ unweighted graph
///////////////////////////////////////////////////////
/**Weighted Graph class. Here we have methods that deal with the fact is weighted or not. these 
 *methods are inheritted by Graph when used. The methods are inheritted by graph.
*/
template<class VERTEX, class WEIGHT=int, class Base=BaseGraph<VERTEX, WEIGHT> >
class WG: public virtual Base{


  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  typedef typename Base::VI   VI;   ///<VI Vertex Iterator
  typedef typename Base::CVI  CVI;  ///<CVI Constant Vertex Iterator
  typedef typename Base::RVI  RVI;  ///<RVI Reverse Vertex Iterator
  typedef typename Base::CRVI CRVI; ///<CRVI Constant Reverse Vertex Iterator

  typedef typename Base::EI   EI;   ///<EI Edge Iterator
  typedef typename Base::CEI  CEI;  ///<CEI Constant Edge Iterator
  typedef typename Base::REI  REI;  ///<REI Reverse Edge Iterator
  typedef typename Base::CREI CREI; ///<CREI Constant Reverse Edge Iterator

  public:
 /**Check if the graph is weighted or not. This function for this class will return 
  *one alll the time. there is a similar function in NWG that will return 0 all the time.
  */ 
 inline int check_weighted() const {
  return 1;
 }

 /**Check if there is any edge connected from vid1 to vid2.
 *@return false if vid1 or vid2 are not in graph, or
 *there is no edge from vid1 to vid2.
 */
  bool IsEdge(VID _v1id, VID _v2id)  {  
    return (Base::IsEdge(_v1id,_v2id) );
  }   
 /**Check if there is any edge connected from v1 to v2.
  *Here v1 is any vertex contains user data in the first parameter,
  *and v2 is any vertex contains user data in the second parameter.
  *if there are more than one, then the first will be applied.
  *
  *@return false if v1 or v2 are not in graph, or
  *there is no edge from v1 to v2.
  */
 bool IsEdge(VERTEX& _v1, VERTEX& _v2)  {
    VI v1;
    EI e12;
    return (Base::IsEdge(_v1,_v2,&v1,&e12) );
  }
  /**Check if there is any edge connected from vid1 to vid2 of specified weight.
   *@return false if vid1 or vid2 are not in graph, or
   *there is no edge from vid1 to vid2.
   */
  bool IsEdge(VID _v1id, VID _v2id, WEIGHT _weight)  {
    VI v1;
    EI e12;
    return ( Base::IsEdge(_v1id,_v2id,_weight,&v1,&e12) );
  }

  /**Check if there is any edge connected from v1 to v2 of specified weight.
   *Here v1 is any vertex contains user data in the first parameter,
   *and v2 is any vertex contains user data in the second parameter.
   *if there are more than one, then the first will be applied.
   *
   *@return false if v1 or v2 are not in graph, or
   *there is no edge from v1 to v2.
   */
  bool IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    VI v1;
    EI e12;
    return ( Base::IsEdge(_v1,_v2,_weight,&v1,&e12) );
  }

 /**Check if there is any edge connected from vid1 to vid2.
 *@return false if vid1 or vid2 are not in graph, or
 *there is no edge from vid1 to vid2.Also the _vi will point to the
 *vertex 1 location and _ei will point to the actual edge;
 */
  bool IsEdge(VID _v1id, VID _v2id,VI* _vi,EI* _ei) {
    return (Base::IsEdge(_v1id,_v2id,_vi,_ei));
  }


  /**Get weight of edge (vid1->vid2).
   *if no such edge WEIGHT(-1) will be returned.
   */
  WEIGHT GetEdgeWeight(VID _v1id, VID _v2id) {
    VI v1;
    EI e12;
    if (Base::IsEdge(_v1id,_v2id,&v1,&e12)) {
      return  e12->weight;
    } else {
      return WEIGHT(-1);
    }
  }

#ifdef _PGRAPH
  /**Get weight of edge (vid1->vid2).
   *if no such edge WEIGHT(-1) will be returned.
   */
  WEIGHT GetEdgeIdWeight(VID _v1id, int _edgeid) {
    VI v1;
    EI e12;
    if (Base::IsEdgeId(_v1id,_edgeid,&v1,&e12)) {
      return  e12->weight;
    } else {
      return WEIGHT(-1);
    }
  }
#endif

  /**Get weight of edge (_v1->_v2).
   *if no such edge WEIGHT(-1) will be returned.
   */
  WEIGHT GetEdgeWeight(VERTEX& _v1, VERTEX& _v2)  {
    return GetEdgeWeight( GetVID(_v1), GetVID(_v2) );
  }

  protected:
  
};
/**Non weighted class. This class will be inheritted if we don't want to use 
 *weights for edges. The weights are still in the graph and they are of type int
 *but the user doesn't have to be concerned with them. Also some methods related with 
 *weight are not implemented in this class as opposed to WG.The methods are inheritted by graph.
*/
template<class VERTEX, class WEIGHT=int, class Base=BaseGraph<VERTEX, WEIGHT> >
class NWG:public virtual Base{


  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  typedef typename Base::VI   VI;   ///<VI Vertex Iterator
  typedef typename Base::CVI  CVI;  ///<CVI Constant Vertex Iterator
  typedef typename Base::RVI  RVI;  ///<RVI Reverse Vertex Iterator
  typedef typename Base::CRVI CRVI; ///<CRVI Constant Reverse Vertex Iterator

  typedef typename Base::EI   EI;   ///<EI Edge Iterator
  typedef typename Base::CEI  CEI;  ///<CEI Constant Edge Iterator
  typedef typename Base::REI  REI;  ///<REI Reverse Edge Iterator
  typedef typename Base::CREI CREI; ///<CREI Constant Reverse Edge Iterator

  public:
/**Check if the graph is weighted or not. This function for this class will return 
  *zero all the time. There is a similar function in WG that will return 1 all the time.
  */ 
  inline int check_weighted() const {
    return 0;
  }

  /**Check if there is any edge connected from vid1 to vid2.
  *@return false if vid1 or vid2 are not in graph, or
  *there is no edge from vid1 to vid2.
  */
  bool IsEdge(VID _v1id, VID _v2id) {
    VI v1;
    EI e12;
    return (Base::IsEdge(_v1id,_v2id,&v1,&e12) );
  }

  /**Check if there is any edge connected from v1 to v2.
  *Here v1 is any vertex contains user data in the first parameter,
  *and v2 is any vertex contains user data in the second parameter.
  *if there are more than one, then the first will be applied.
  *
  *@return false if v1 or v2 are not in graph, or
  *there is no edge from v1 to v2.
  */
  bool IsEdge(VERTEX& _v1, VERTEX& _v2)  {
    VI vi;
    EI e12;
    return (Base::IsEdge(_v1,_v2,&vi,&e12) );
  }
  
  /**Check if there is any edge connected from vid1 to vid2.
   *@return false if vid1 or vid2 are not in graph, or
   *there is no edge from vid1 to vid2.Also the _vi will point to the
   *vertex 1 location and _ei will point to the actual edge;
   */
  bool IsEdge(VID _v1id, VID _v2id,VI* _vi,EI* _ei) {
    return (Base::IsEdge(_v1id,_v2id,_vi,_ei));
  }

};


////////////////////////////////////////////////////////
//
// multi/ non multi graph
///////////////////////////////////////////////////////
/**Multiple Edge Graph class. Here for right now we have only chacks but in the feature
 *new methods related with multiplicity will be added. These methods are inheritted by graph.
*/
template<class VERTEX, class WEIGHT=int, class Base=BaseGraph<VERTEX, WEIGHT> >
class MG: public virtual Base{
  public:
  /**
   *return 1 all the time; the same function is implemented in NMG and there returns
   *zero all the time. 
   */
  inline int check_multi() const {
    return 1;
  }
  /**
   *This function returns 1 all the time since the graph will allow multiple edges. 
   */
  inline int check_edge(VID v1, VID v2) const {
    //cout<<"M::check_edge"<<endl;
    return 1;
  }
  /**
   *This function returns 1 all the time since the graph will allow multiple edges. 
   */
  inline int check_edge(VERTEX& v1, VERTEX& v2) const {
    //cout<<"M::check_edge"<<endl;
    return 1;
  }
};

/**Non Multiple Edge Graph class. Here for right now we have only chacks but in the feature
 *new methods related with multiplicity will be added. These methods are inheritted by graph.
*/
template<class VERTEX, class WEIGHT=int, class Base=BaseGraph<VERTEX, WEIGHT> >
class NMG: public virtual Base{
  public:

  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  typedef typename Base::VI   VI;   ///<VI Vertex Iterator
  typedef typename Base::CVI  CVI;  ///<CVI Constant Vertex Iterator

  typedef typename Base::EI   EI;   ///<EI Edge Iterator
  typedef typename Base::CEI  CEI;  ///<CEI Constant Edge Iterator
  
  /**
   *return 0 all the time; the same function is implemented in MG and there returns
   *one all the time. 
   */
  inline int check_multi() const{
    return 0;
  }

  /**
   *This function check if the edge _v1->_v2 exist already in the graph and returns 
   *zero if not and one if the edge is in the graph.
   */
  inline int check_edge(VID _v1, VID _v2){
    //cout<<"NM::check_edge"<<endl;
    VI v1;
    EI e12;
    if(_v1 == _v2) return 0;
    if(Base::IsEdge(_v1, _v2, &v1,&e12)) return 0;
    else return 1;
  }
  /**
   *This function check if the edge _v1->_v2 exist already in the graph and returns 
   *zero if not and one if the edge is in the graph.
   */
  inline int check_edge(VERTEX& _v1, VERTEX& _v2){
    //cout<<"NM::check_edge"<<endl;
    VI v1;
    EI e12;
    VI vi1,vi2;
    if(!IsVertex(_v1,&vi1)) return 0;
    if(!IsVertex(_v2,&vi2)) return 0;
    if(vi1->vid == vi2->vid) return 0;
    if(Base::IsEdge(_v1,_v2,&v1,&e12)) return 0;
    else return 1;
  }
};


////////////////////////////////////////////////////////
//
//    the graph class
///////////////////////////////////////////////////////
/**The Graph class that will be instantiated by the user.
 *It is templated with three arguments. The first one describes the directness,
 *the second describe if the graph will be with multiple edges or not
 *and the last describes if the graph will weighted or not. In other
 *words D can be one of DG (DirectedGraph) or
 *UG (UndirectedGraph), M can be one of MG (MultipleGraph- allows
 *multiple edges) or NMG( non multiple edges) and W can be
 *either WG(methods related with weight enabled) or
 *NWG (methods related with weights disabled(missing); if the user will try to use them for
 *non-weighted graphs it will generate compiler errors);
*/
template <class D, class M, class W, class VERTEX,class WEIGHT=int>
class Graph:public D, public M, public W{
  public:


  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;


  typedef  VERTEX VERTEX_TYPE;
  typedef  WEIGHT WEIGHT_TYPE ;
  
  typedef typename D::VI   VI;   ///<VI Vertex Iterator
  typedef typename D::CVI  CVI;  ///<CVI Constant Vertex Iterator
  typedef typename D::RVI  RVI;  ///<RVI Reverse Vertex Iterator
  typedef typename D::CRVI CRVI; ///<CRVI Constant Reverse Vertex Iterator

  typedef typename D::EI   EI;   ///<EI Edge Iterator
  typedef typename D::CEI  CEI;  ///<CEI Constant Edge Iterator
  typedef typename D::REI  REI;  ///<REI Reverse Edge Iterator
  typedef typename D::CREI CREI; ///<CREI Constant Reverse Edge Iterator


  typedef VI iterator;
  typedef CVI const_iterator;
  typedef VERTEX value_type;

  /**@name Constructors and Destructor*/
  //@{

  /**Constrcutor. Do nothing.
   *@note this constrcutor didn't  reserve any space for verts or edges
   */
  Graph(){}
  /**Constrcutor. 'reserve' space for vertices.
   *@param int how many vertices will be reserved.
   */
  Graph(int _sz) : D(_sz){}
  /**Constrcutor. 'reserve' space for vertices.
   *@param _sz how many vertices will be reserved.
   *@param _edgelistsz how many edges will be reserved.
   */
  Graph(int _sz,int _edgelistsz) : D(_sz,_edgelistsz){}
  ~Graph(){
    //????????????????
    //this->EraseGraph();
  }
  //@}

  //the addVertex methods will be inherited from weighted multi di graph
  //the addEdge methods have to be modified to chech for multiplicity/ weight 

  /**
   *@return Iterator to the first vertex of the graph;
   */
  VI begin(){
    return this->v.begin();
  }
  /**
   *@return an interator that points immediately after the last vertex of the graph.
   */
  VI end(){
    return this->v.end();
  }

  /**
   *@return const iterator to the first vertex of the graph;
   */
  CVI begin() const {
    return this->v.begin();
  }
  /**
   *@return an const interator that points immediately after the last vertex of the graph.
   */
  CVI end() const {
    return this->v.end();
  }

  /**
   *@return the id of the last node inserted in the graph +1 
   */
  inline int getVertIDs(){
    return this->vertIDs;
  }
  /**
   *Set an internal variable such that when a new vertex is added it's id will be _vids;
   */
  inline void setVertIDs(int _vids){
    this->vertIDs = _vids;
  }
  /**
   *Check if the graph is directed or not; 
   *@return 1 if the graph is directed zero if the graph is undirected; 
   */
  inline int IsDirected() const {
    return D::check_directed();
  }
  /**
   *Check if the graph is weighted or not; 
   *@return 1 if the graph is weighted zero if the graph is unweighted; 
   */
  inline int IsWeighted() const {
    return W::check_weighted();
  }
  
  /**
   *Check if the graph support multiple edges or not; 
   *@return 1 if the graph supports and zero if the graph doesn't support; 
   */
  inline int IsMulti() const {
    return M::check_multi();
  }
  /**
   *@return the number of vertices in the graph
   */
  inline int size() const {
    return this->v.size();
  }
  /**
   *@return the number of edges in the graph
   */
  inline int GetEdgeCount() const{
    if(D::check_directed()) return this->numEdges;
    else return this->numEdges/2;
  }
  /**
   *Add an edge into a graph. The functions takes care about directness, multiplicity,
   *weightness. If the graph is unweighted the user doesn't have to specify the weight.
   *@param _v1 the first vertex(user data)
   *@param _v2 the second vertex(user data)
   *@param _weight the weight of the edge.
   *@note The weight of the edge doesn't haveto be specified if the graph is unweighted. 
   *@return OK/ERROR depending if the operation succeded or not
   */
  inline int AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight=WEIGHT(-1)) {
    //cout<<"Graph::AddEdge"<<endl;
    if(M::check_edge(_v1,_v2)) return D::AddEdge(_v1,_v2,_weight);
    else return ERROR;
  }
  /**
   *Add an edge into a graph. The functions takes care about directness, multiplicity,
   *weightness. If the graph is unweighted the user doesn't have to specify the weight.
   *@param _v1 the first vertex
   *@param _v2 the second vertex
   *@param _weight the weight of the edge.
   *@note The weight of the edge doesn't haveto be specified if the graph is unweighted. 
   *@return OK/ERROR depending if the operation succeded or not
   */
  inline int AddEdge(VID _v1, VID _v2, WEIGHT _weight=WEIGHT(-1)) {
    //cout<<"Graph::AddEdge"<<endl;
    //cout<<"directness "<<D::isDirected()<<endl;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
    if(M::check_edge(_v1,_v2)) return D::AddEdge(_v1,_v2,_weight);
    //return D::AddEdge(_v1,_v2,_weight);
    else return ERROR;
  }
  

  /**
   *Get_kth_Edge : This function is used in RNA folding
   *Get the kth Edge in the graph
   *@param k Index of the Edge to be found
   *@note will exit if k > number of edges 
   *@return pair<Vertex1ID,Vertex2ID> corresponding to the kth edge
   */
  pair<VID,VID> Get_kth_Edge(int k)
  {
    pair<VID,VID> p;
    int n=size();//=Graph Size
    int count=0;
    
    VID i;
    for(i=0;  i<n; ++i)
    {
        count+=this->v[i].GetEdgeCount();
        if(count>=k)
        {
            break; 
        }
    }
    
    if(i==n)
    {
        cout<<" FATAL ERROR Get_kth_Edge("<<k<<") : Value of k["<<k<<"] > Total number of edges["<<count<<"]"<<endl;
        cout<<" Program Exiting "<<endl;
        exit(1); 
    
    }
    
    count=count-this->v[i].GetEdgeCount();
    
    VID m=k-count;
    
    p.first=this->v[i].vid;
    p.second=this->v[i].edgelist[m-1].vertex2id;
    
    return p;
  }

  /**
   *This functions is for weighted graphs. 
   *This functions has different functionality for Directed and Undirected graphs.
   *If the graph is directed this function will add two edges into the graph. The first
   *one is from _v1 to _v2 and has the weight _p.first and the second one is from 
   *_v2 to _v1 and has weight _p.second. for the undirected graphs this function will add 
   *one edge  from _v1 to _v2 but the weight associated with this edge is different 
   *depending on the way you traverse the edge.  
   *@param _v1 the first vertex
   *@param _v2 the second vertex
   *@param _p a pair of weights for the edge(edges) that will be added.
   *@return OK/ERROR depending if the operation succeded or not
   */
  int  AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _p){
    //if(AddEdge(_v1,_v2,p.first) == -1) return -1;
    //return AddEdge(_v2,_v1,p.second); 
    //!!!!! the correct way of doing this ???????????????????????????
    if(M::check_edge(_v1,_v2)&&M::check_edge(_v2,_v1)) return D::AddEdge(_v1,_v2,_p);
    //else {cout<<"ERROR"<<endl;return ERROR;}
    else return ERROR;
  }

  /**
   *This functions is for weighted graphs. 
   *This functions has different functionality for Directed and Undirected graphs.
   *If the graph is directed this function will add two edges into the graph. The first
   *one is from _v1 to _v2 and has the weight _p.first and the second one is from 
   *_v2 to _v1 and has weight _p.second. for the undirected graphs this function will add 
   *one edge  from _v1 to _v2 but the weight associated with this edge is different 
   *depending on the way you traverse the edge.  
   *@param _v1 the first vertex(user data)
   *@param _v2 the second vertex(user data)
   *@param _p a pair of weights for the edge(edges) that will be added.
   *@return OK/ERROR depending if the operation succeded or not
   */
  int  AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _p){
    //cout<<D::isDirected()<<endl;
    //if(AddEdge(_v1,_v2,p.first) == -1) return -1;
    //return AddEdge(_v2,_v1,p.second); 
    if(M::check_edge(_v1,_v2)&&M::check_edge(_v2,_v1)) return D::AddEdge(_v1,_v2,_p);
    //else {cout<<"ERROR"<<endl;return ERROR;}
    else return ERROR;
  }

  /**Connect vertices listed in given vector<VID> with uniform wright.
   *If path contains (vid1,vid2,vid3,...), then
   *edges (vid1,vid2), (vid2,vid3), ... will be added to graph.
   *All of these new edges will have same weight WEIGHT.
   *@param _path the vector of vertices(user data)
   *@param WEIGHT the weight for all new edges. This doesn't have to be specified for 
   *unweighted graphs.
   *@note ERROR will be returned if one of vid in the path
   *could not be found in the graph. 
   *@return The number of edges added succesfully.
   */
  int AddPath( vector<VID>& _path, WEIGHT _wt=WEIGHT(-1)) {
    int i;
    int cnt=0;
    for (i = 0; i < _path.size(); i++){
        if (!this->IsVertex(_path[i])) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
    if(AddEdge(_path[i],_path[i+1],_wt)!=ERROR) cnt++;
    }
    return cnt;
  }

  /**Connect vertices listed in given vector<VERTEX> with uniform weight.
   *If path contains (v1,v2,v3,...), then
   *edges (v1,v2), (v2,v3), ... will be added to graph.
   *All of these new edges will have same weight WEIGHT.
   *@param _path the vector of vertices(user data)
   *@param WEIGHT the weight for all new edges. This doesn't have to be specified for 
   *unweighted graphs.
   *@note if vi is not found in graph then a new vertex will be added.
   *@return The number of edges added succesfully.
   */
  int AddPath( vector<VERTEX>& _path, WEIGHT _wt=WEIGHT(-1)) {
    int i;
    int cnt;
    if (!this->IsVertex(_path[0])) AddVertex(_path[0]);
    for (i = 0; i < _path.size() - 1; i++){
      if (!this->IsVertex(_path[i+1])) AddVertex(_path[i+1]);
      if(AddEdge(_path[i],_path[i+1],_wt) != ERROR) cnt++;
    }
    return cnt;
  }

  //the following AddPath methods are only for weighted graph;
  
  /**This method is only for weighted graphs.
   *Connect vertices listed in given vector<VID> with various weight.
   *If path contains (vid0, vid1,vid2,vid3,...), then
   *edges (vid0,vid1), (vid1,vid2), (vid2,vid3), ... will be added to
   *graph with weight vector[0].second, vector[1].second, vector[2].second,..
   *
   *@param vector< pair<VID,WEIGHT> >& a list of vids and weights.
   *@note ERROR will be returned if one of vid in the path
   *could not be found in the graph.
   *@return The number of edges added succesfully.
   */
  /*
  int AddPath( vector< pair<VID,WEIGHT> >& _path) {
    int i;
    int cnt = 0;
    for (i = 0; i < _path.size(); i++){
        if (!this->IsVertex(_path[i].first)) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
        if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second)!=ERROR) cnt++;
    }
    return cnt;
  }
  */

  /**This method is only for weighted graphs.
   *It connect vertices listed in given vector<VID> with various weights.
   *If path contains (v0, v1,v2,v3,...), then
   *edges (v0,v1), (v1,v2), (v2,v3), ... will be added to
   *graph with weight vector[0].second, vector[1].second, vector[2].second,..
   *
   *@param vector< pair<VERTEX,WEIGHT> >& a list of user data and weights.
   *@note if vi is not found in graph then a new vertex will be added.
   *@return The number of edges added succesfully.
   */
  /*
  int AddPath( vector< pair<VERTEX,WEIGHT> >& _path) {
    int i;
    int cnt;
    if (!IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (i = 0; i < _path.size() - 1; i++){
      if (!this->IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
      if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second) != ERROR) cnt++;
    }
    return cnt;
  }
  */
  /**Thsi method is only for weighted graphs.
   *floatly vertices listed in given vector<VID> with various weight.
   *If path contains (vid0, vid1,vid2,vid3,...), then
   *edges (vid0,vid1), (vid1,vid0), (vid1,vid2), (vid2,vid1), 
   *(vid2,vid3), (vid3,vid2) ... will be added to
   *graph with weight vector[0].second.first, vector[0].second.second
   *vector[1].second.first, vector[1].second.second,
   *vector[2].second.first,vector[2].second.second..
   *
   *@param vector< pair<VID, pair<WEIGHT,WEIGHT> > >& a list of vids and weights.
   *@note ERROR will be returned if one of vid in the path
   *could not be found in the graph.
   *@return The number of edges added succesfully.
   */
  /*
  int AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& _path) {
    int i;
    int cnt=0;
    for (i = 0; i < _path.size(); i++){
        if (!this->IsVertex(_path[i].first)) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
        if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second)!=ERROR) cnt++;
    }
    return cnt;
  }
  */
  /**floatly vertices listed in given vector<V> with various wright.
   *If path contains (v0, v1,v2,v3,...), then
   *edges (v0,v1), (v1,v0), (v1,v2), (v2,v1), 
   *(v2,v3), (v3,v2) ... will be added to
   *graph with weight vector[0].second.first, vector[0].second.second
   *vector[1].second.first, vector[1].second.second,
   *vector[2].second.first,vector[2].second.second..
   *
   *@param vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& a list of user data and weights.
   *@note if vi is not found in graph then a new vertex will be added.
   *@return The number of edges added succesfully.
   */
  int AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& _path) {
    int i;
    int cnt=0;
    if (!this->IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (i = 0; i < _path.size() - 1; i++){
      if (!this->IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
      if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second)!=ERROR) cnt++;
    }
    return cnt;
  }

  /**Get All edges in this graph.
   *@param edges vector where the edges are stored
   *@return A edge list. One edge is defined as 2 VIDs.
   *@note This methods works both for weighted and unweighted graphs.
   */
  int GetEdges(vector< pair<VID,VID> >& edges) const {
    //vector< pair<VID,VID> > edges;    
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
    if(!(IsDirected()) && 
       (vi->vid > ei->vertex2id)) continue;
    pair<VID,VID> newedge(vi->vid, ei->vertex2id);
    edges.push_back( newedge );
      }
    }
    return edges.size();
  }
  
  /**Get All edges in this graph.
   *@param edges vector where the edges are stored
   *@return A edge list. One edge is defined as 2 VERTEX data structure(user data).
   *@note This methods works both for weighted and unweighted graphs.
   */
  /*
  int GetEdgesVData(vector< pair<VERTEX,VERTEX> >& edges) const {
    //vector< pair<VERTEX,VERTEX> > edges; 
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
    if(!(IsDirected()) && 
       (vi->vid > ei->vertex2id)) continue;
    VERTEX v2data = GetData(ei->vertex2id);
    pair<VERTEX,VERTEX> newedge(vi->data, v2data);
    edges.push_back( newedge );
      }
    }
    return edges.size();
  }
  */
  /**Get All edges in this graph.
   *@param edges vector where the edges are stored
   *@return A edge list. One edge is defined as 2 VIDs and weight.
   *@note This methods works only for weighted graphs.
   */
  int  GetEdges(vector< pair< pair<VID,VID>, WEIGHT> >& edges) const  {
    //vector< pair< pair<VID,VID>, WEIGHT> > edges;    
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
    if(!(IsDirected()) && 
       (vi->vid > ei->vertex2id)) continue;
    pair<VID,VID> newedge(vi->vid, ei->vertex2id);
    pair<pair<VID,VID>,WEIGHT> newedgewt(newedge, ei->weight);
    edges.push_back( newedgewt );
      }
    }
    return edges.size();
  }
  /**Get All edges in this graph.
   *@param edges vector where the edges are stored
   *@return A edge list. One edge is defined as 2 VERTEX data structure(user data).
   *@note This methods works only for weighted graphs.
   */
  int GetEdgesVData(vector< pair< pair<VERTEX,VERTEX>, WEIGHT> >& edges) const  {
    //vector< pair< pair<VERTEX,VERTEX>, WEIGHT> > edges; 
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
    if(!(IsDirected()) && 
       (vi->vid > ei->vertex2id)) continue;
    VERTEX v2data = ((BaseGraph<VERTEX,WEIGHT>*)(this))->GetData(ei->vertex2id);

    pair<VERTEX,VERTEX> newedge(vi->data, v2data);
    pair<pair<VERTEX,VERTEX>,WEIGHT> newedgewt(newedge, ei->weight);
    edges.push_back( newedgewt );
      }
    }
    return edges.size();
  }
  
  /**This method works only for weighted graphs. 
   *Change edge's (defined be vid1 and vid2) weight to WEIGHT.
   *@param WEIGHT edge's new weight
   *@note this method calls DeleteEdge and AddEdge
   *@return ERROR if DeleteEdge returned ERROR or AddEdge returned ERROR.
   *OK if AddEdge returned OK
   *@see DeleteWtEdge(VID, VID, WEIGHT, int _n) and AddEdge(VID, VID, WEIGHT)
   */
  int ChangeEdgeWeight(VID _v1id, VID _v2id, WEIGHT _weight) {
    //this is only for weighted one;
    //something such that the compiler will generate errors
    //or return -1 immediate
    //and I think this can be done faster; and moved in weighted class!!!
    if(!W::check_weighted()) return ERROR;
    if ( D::DeleteEdge(_v1id, _v2id) == OK) {
      return AddEdge(_v1id, _v2id,_weight);
    } else {
      return ERROR;
    }  
  }

  /**This method works only for weighted graphs. 
   *Change edge's (defined be VERTEX1 and VERTEX2) weight to WEIGHT.
   *@param WEIGHT edge's new weight
   *@note this method calls DeleteEdge and AddEdge
   *@return ERROR if DeleteEdge returned ERROR or AddEdge returned ERROR.
   *OK if AddEdge returned OK
   *@see DeleteWtEdge(VERTEX&, VERTEX&, WEIGHT, int _n) and 
   *AddEdge(VERTEX&, VERTEX&, WEIGHT)
   */
  int ChangeEdgeWeight(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    if(!W::check_weighted()) return ERROR;
    if ( DeleteEdge(_v1, _v2) == OK) {
        return AddEdge(_v1, _v2,_weight);
    } else {
      return ERROR;
    }  
  } 

  /**@name I/O Methods*/
  //@{
  
  //IO stuff
  void 
  ReadDotGraph(const char*  _fname) {
    
    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadDotGraph(myifstream);
    myifstream.close();
  }

  void 
  ReadDotGraph(istream& _myistream) {
    VID v1id, v2id, maxVID;
    CVI  cv1;
    //VI  v1;
    VERTEX data;
    WEIGHT weight;
    int nVerts=0, nEdges=0;
    char tagstring[100];
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTART") ) {
        cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
        return;
    }
    
    if (this->numVerts != 0) {
        D::EraseGraph(); // empty graph before filling it in
    }
    _myistream >> v1id;
    while(v1id != -1){
      AddVertex(data,v1id);
      nVerts++;
      if ( !IsVertex(v1id,&cv1) ) {
    cout << "\nIn ReadGraph: didn't add v1...";
    //return -1;
      }
      _myistream >> v1id;
    }

    _myistream >> v1id;
    while(v1id != -1){

      _myistream >> v2id;
      if(AddEdge(v1id,v2id,weight) < 0) {
    cout<<"Error while trying to insert edge "<<v1id<<" "<<v2id<<endl;
      }
      nEdges++;
      _myistream >> v1id;
    }
    maxVID = nVerts;
    this->numVerts = nVerts;

    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;
    //this->numEdges = nEdges;
    this->vertIDs = maxVID; // set the maximum VID used so far...
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }
  }

  void 
  ReadExoGraph(const char*  _fname) {
    
    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadExoGraph(myifstream);
    myifstream.close();
  }

  void 
  ReadExoGraph(istream& _myistream) {
    VID v1id, v2id, maxVID;
    VI  v1;
    VERTEX data;
    WEIGHT weight;
    int nVerts, nEdges, nedges;
    int tr;
    float dtr;
    char tagstring[100];
    int i;

    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTART") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
      return;
    }
    
    if (this->numVerts != 0) {
      this->EraseGraph(); // empty graph before filling it in
    }
    
    _myistream >> nVerts >> nEdges >> maxVID;

    //first add vertices; our graph checks when an edge 
    //is added for both source and destination;
    for (i = 0; i < nVerts; i++){
      this->AddVertex(data,i);
      if ( !IsVertex(i,&v1) ) {
    cout << "\nIn ReadGraph: didn't add v1...";
      }
    }
    
    for (i = 0; i < nVerts; i++){
      _myistream >> v1id ;             // read and add vertex 
      _myistream >> tr >> dtr >>dtr >>dtr >> tr;
      //AddVertex(data,v1id);
      if ( !IsVertex(v1id,&v1) ) {
    cout << "\nIn ReadGraph: didn't add v1...";
      }
      
      _myistream >> nedges;               // read and add its edges
      for (int j = 0; j < nedges; j++){
    if(W::check_weighted()){
      _myistream >> v2id;
      _myistream >> tr;
    }
    else _myistream >> v2id;//read only the id and put a default weight   
    this->AddEdge(v1id,v2id,weight);
      }
    }
    
    this->numVerts = nVerts;
    //internally we keep trace to all the edges in the graph;
    //which for undirected is float
    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;
    this->vertIDs = maxVID; // set the maximum VID used so far...
    // should sort verts & find biggest used...
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }



  }

  // end gti
 
  /**Display every vertex and edge information in the graph.
   *@see WtVertexType::DisplayEdgelist
   */
  void DisplayGraph() const {
    CVI vi;
    int i;
    cout<<endl;
    for (vi = this->v.begin(), i=0; vi < this->v.end(); vi++, i++) {
      cout << setw(3) << i << ": ";
      vi->DisplayEdgelist(W::check_weighted());
      cout<<endl;
    }
  }

  /**Display every vertex and edge information in the graph to an ostream
   *@see WtVertexType::DisplayEdgelist
   */
  void DisplayGraph(ostream& s) const {
    CVI vi;
    int i;
    s<<endl;
    for (vi = this->v.begin(), i=0; vi < this->v.end(); vi++, i++) {
      s << setw(3) << i << ": ";
      vi->DisplayEdgelist(s,W::check_weighted());
      s<<endl;
    }
  }

  /**Display every vertex information in the graph.
   *This method calls DisplayVertex(VID) iterately.
   *@see DisplayVertex(VID)
   */
  void DisplayVertices() const {
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      DisplayVertex(vi->vid); 
    } 
  }

  /**Display vertex of VID in the graph.
   *This method outputs VID and user data in this vertex.
   *@note if this VID is not found in the graph, an error
   *message will be print out.
   */
  void DisplayVertex(VID _v1id) const{
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      cout << "vertex: id =" << setw(3) << _v1id; 
      cout << ", data = [" << v1->data;
      cout << "]";
      cout<< endl;
    } else {
      cout << "vertex with id=" << _v1id << " not in graph.";
    }
  }

  /**Display vertex of VID and its edge information in the graph.
   *@see DisplayEdgelist
   *@note if this VID is not found in the graph, an error
   *message will be print out.
   */
  void DisplayVertexAndEdgelist(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      cout << "vertex: ";
      v1->DisplayEdgelist(W::check_weighted());
    } else {
      cout << "vertex with id=" << _v1id << " not in graph.";
    }
  }
  
  /**Write graph info to the given output stream.
   *This method outputs numVerts, numEdges, vertIDs
   *to the output stream and calls 
   *WtVertexType::WriteEdgelist for each vertex in 
   *this graph.
   *@see WtVertexType::WriteEdgelist
   */
  void WriteGraph(ostream& _myostream) const {
  
#ifdef _ASCI_
    _myostream << endl << "GRAPHSTART";
#else
    _myostream << endl << "#####GRAPHSTART#####";
#endif
    if(this->IsDirected())
    _myostream << endl << this->numVerts << " " << this->numEdges << " " << this->vertIDs; 
    else 
    _myostream << endl << this->numVerts << " " << this->numEdges / 2 << " " << this->vertIDs; 
 
    //format: VID VERTEX #edges VID WEIGHT VID WEIGHT ... 
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      _myostream << endl;
      vi->WriteEdgelist(_myostream,W::check_weighted());
    } 
    
#ifdef _ASCI_
    _myostream << endl << "GRAPHSTOP";
#else
    _myostream << endl << "#####GRAPHSTOP#####";
#endif
    _myostream << endl; 
  }
  
  /**Write graph info to the file of file name, _filename.
   *This method calls WriteGraph(ostream& _myostream).
   */
  void WriteGraph(const char* _fname) const { 
    ofstream  myofstream(_fname);
    if (!myofstream) {
      cout << "\nInWriteGraph: can't open outfile: " << _fname ; 
    }
    WriteGraph(myofstream);
    myofstream.close();
  }

  void WriteMetis(ostream& _myostream) const {
    
    int ne =0;
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for(CEI ei = vi->edgelist.begin();ei!=vi->edgelist.end();ei++){
    ne++;
      }
    }

    _myostream<<this->GetNumVerts()<<" "<<ne/2;
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      _myostream<<endl;

      for(CEI ei = vi->edgelist.begin();ei!=vi->edgelist.end();ei++){
    _myostream<<ei->vertex2id+1<<" ";
      }

    } 
  }

  /**Write graph info to the file of file name, _filename.
   * the format is metis format;
   */
  void WriteMetis(const char* _fname) const { 
    ofstream  myofstream(_fname);
    if (!myofstream) {
      cout << "\nInWriteGraph: can't open outfile: " << _fname ; 
    }
    WriteMetis(myofstream);
    myofstream.close();
  }

  void WriteDotGraph(ostream& _myostream) const {
    
    int ne =0;
    ofstream  dotstream("gen.dot");
    
    _myostream<<"GRAPHSTART"<<endl;
    dotstream<<"digraph SCC {"<<endl<<"node[shape=circle];"<<endl<<"ratio=0.5"<<endl;
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      _myostream<<vi->vid<<endl;
      dotstream<<vi->vid<<endl;
    }
    _myostream<<"-1"<<endl;
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for(CEI ei = vi->edgelist.begin();ei!=vi->edgelist.end();ei++){
    _myostream<<vi->vid<<" "<<ei->vertex2id<<endl;
    dotstream<<vi->vid<<"->"<<ei->vertex2id<<endl;
      }
    }
    _myostream<<"-1"<<endl;
    dotstream<<"}"<<endl;
    _myostream<<"GRAPHSTOP"<<endl;
    dotstream.close();
  }

  /**Write graph info to the file of file name, _filename.
   * the format is metis format;
   */
  void WriteDotGraph(const char* _fname) const { 
    ofstream  myofstream(_fname);
    if (!myofstream) {
      cout << "\nInWriteGraph: can't open outfile: " << _fname ; 
    }
    WriteDotGraph(myofstream);
    myofstream.close();
  }


  /**Read graph info from the given input stream. Assign vid with given values.
   *Read data which were written by WriteGraph.
   @note Error message will be outputed if something wrong
   *during processing.
   */
  void ReadGraph(istream& _myistream) {
    VID v1id, v2id, maxVID;
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
      this->EraseGraph(); // empty graph before filling it in
    }
    
    _myistream >> nVerts >> nEdges >> maxVID;
    
    for (int i = 0; i < nVerts; i++){
      _myistream >> v1id >> data;             // read and add vertex 
      AddVertex(data,v1id);
      if ( !IsVertex(v1id,&v1) ) {
    cout << "\nIn ReadGraph: didn't add v1...";
      }
      
      _myistream >> nedges;               // read and add its edges
      for (int j = 0; j < nedges; j++){
    if(W::check_weighted())
      _myistream >> v2id >> weight; 
    else _myistream >> v2id;//read only the id and put a default weight   
    v1->AddEdge(v2id,weight);
      }
    }
    
    this->numVerts = nVerts;
    //internally we keep trace to all the edges in the graph;
    //which for undirected is float
    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;
    this->vertIDs = maxVID; // set the maximum VID used so far...
    // should sort verts & find biggest used...
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }
  }

  /**Read graph info from the file of file name, _filename.
   *This method calls ReadGraph(istream& _myistream).
   */
  void ReadGraph(const char*  _fname) {
    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadGraph(myifstream);
    myifstream.close();
  }

  /** The same as Graph::ReadGraph() but 
   *automatic assign vid to each nodes, need to transform vid in the edgelist  
   */
  void ReadGraphwithAutoVID(istream& _myistream) {
    //this is not tested ?
    VID v1id, v2id, maxVID;
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
      this->EraseGraph(); // empty graph before filling it in
    }
    
    _myistream >> nVerts >> nEdges >> maxVID;
    
    for (int i = 0; i < nVerts; i++){
      _myistream >> data;             // read and add vertex 
      AddVertex(data);
      v1id = i;       //start vid from 0
      if ( !IsVertex(v1id,&v1) ) {
    cout << "\nIn ReadGraph: didn't add v1...";
      }
      
      _myistream >> nedges;               // read and add its edges
      for (int j = 0; j < nedges; j++){
    _myistream >> v2id >> weight; 
    v1->AddEdge(v2id,weight);
      }
    }
    
    this->numVerts = nVerts;

    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;

    this->vertIDs = maxVID; // set the maximum VID used so far...
    // should sort verts & find biggest used...
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }
  }

  /** The same as Graph::ReadGraph() but 
   *automatic assign vid to each nodes, need to transform vid in the edgelist  
   */
  void ReadGraphwithAutoVID(const char*  _fname) {

    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadGraphwithAutoVID(myifstream);
    myifstream.close();
  }
  //@}
};//end class GRAPH  

};

#endif

/*
1)
virtual vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;
    this function has to be implemented different in W/ NW classes
    vector< <pair<VERTEX,VERTEX>> > GetEdgesVData() const; for NW class
2)
WEIGHT  GetEdgeWeight(VID, VID) const;  
3)
        virtual vector<VID> GetPredecessors(VID) const;
        (also for succesors)
    vector<VID> GetSinks() const;
        vector<VID> GetSources() const;

this methods should be only for directed graph
Undirected graph should not have this methods;
Undirected should have getEdges(VID) instead;
*/
/*  
  vector< pair<VID,VID> >
  GetEdges()  {
    vector< pair<VID,VID> > edges;    
    edges.reserve(this->numEdges);
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
    if (vi->vid < ei->vertex2id){
      pair<VID,VID> newedge(vi->vid, ei->vertex2id);
      edges.push_back( newedge );
    }
      }
    }
    return edges;
  } 

  vector< pair<VERTEX,VERTEX> >
  GetEdgesVData() const {
    vector< pair<VERTEX,VERTEX> > edges;
    edges.reserve(this->numEdges);
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
    if ( vi->vid < ei->vertex2id) {
      VERTEX v2data = GetData(ei->vertex2id);
      pair<VERTEX,VERTEX> newedge(vi->data, v2data);
      edges.push_back( newedge );
    }
      }
    }
    return edges;
  }
*/



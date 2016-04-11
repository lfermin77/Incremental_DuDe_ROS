#include "Graph.h"
#include "Defines.h"
#ifndef GRAPHALGO_H
#define GRAPHALGO_H
///////////////////
//graph algorithms 
//////////////////

namespace graph{

/************************************TO DO

 - GetCCStats can be optimized
 -   in GetCCStats getVertices vid; return value as a parameter;

******************************************/


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
            
            for(int i=0; i<_numVerts+1; i++) {
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


//============================================
//the new graph.h brings new the visitors concept(boost)
//============================================
#define WHITE 0
#define GRAY 1
#define BLACK 2

/** Visitor_base is the class that has to be inherited by every 
* visitor class; The return values for every method is not used 
* in DFS_functor, BFS_functor; It is used in DFS_functor_EQ
* and BFS_functor_EQ. The traversal in the last two cases is canceled if 
* from inside visitor's method you return EARLY_QUIT;
*/
template< class GRAPH> 
class visitor_base{
  /**
   * GRAPH's iterators
   */
  typedef typename GRAPH::VI VI;
  typedef typename GRAPH::EI EI;
 public:
  /**
   * Null constructor
   */
  visitor_base(){}
  /**
   * All the visitors will be related with a graph. This constructor can be used when you
   * nees to store the Graph associated with the current traversal/visitor.
   */
  visitor_base(GRAPH& _g){}

  /**
   *This method of the visitor will be called by the traversal when 
   *a vertex is reached for the first time. 
   */
  virtual inline int vertex(VI _v){return 1;}

  /**
   *This method of the visitor will be called when an edge whose 
   *destination is WHITE(unvisted).
   *@param _v iterator to the starting vertex of the edge 
   *@param _e iterator to the edge(edge data and destination vertex)
   */
  virtual inline int tree_edge(VI _v, EI _e){return 1;}


  /**
   *This method of the visitor will be called when an edge whose 
   *destination is GRAY(visted but not finished).
   *@param _v iterator to the starting vertex of the edge 
   *@param _e iterator to the edge(edge data and destination vertex)
   */
  virtual inline int back_edge(VI _v, EI _e){return 1;}
  
  //forward or traversal edge is the edge with destination vertex black
  /**
   *This method of the visitor will be called when an edge whose 
   *destination is BLACK(finished).
   *@param _v iterator to the starting vertex of the edge 
   *@param _e iterator to the edge(edge data and destination vertex)
   */
  virtual inline int ft_edge(VI _v, EI _e){return 1;}
  /**
   *This method of the visitor will be called by the traversal when 
   *a vertex's color become BLACK. 
   */
  virtual inline int finish_vertex(VI _v){return 1;}
};


/*
Internal function used by BFS_functor
*/
template<class GRAPH>
int _functor_BFS (GRAPH& _g , 
          visitor_base<GRAPH>& _f, 
          VID _startVid,
          map<VID, int>& _color) {
   
  list<VID> q; 
  typename GRAPH::VI v1,v2;
  typename GRAPH::EI ei;

  VID v1id, v2id; 
  //map<VID, int> _color;
    
  if ( _g.IsVertex(_startVid,&v1) ) {
    q.push_back(_startVid);
    _f.vertex(v1);
    _color[_startVid] = GRAY;
  } else {
    cout << "\nIn GraphBFS: root vid=" << _startVid << " not in graph";
    return -1; 
  }
  while ( !q.empty() ) {
    v1id = q.front();
    if ( _g.IsVertex(v1id,&v1) ){
      for (ei = v1->edgelist.begin(); ei < v1->edgelist.end(); ei++) {
    //we handle the second vertex
    v2id = ei->vertex2id;
    if ( _g.IsVertex(v2id,&v2)) { 
      if(_color[v2id] == WHITE){
        //first we handle the vertex
        _f.vertex(v2);
        //and after the edge
        _f.tree_edge(v1,ei);
        q.push_back(v2id);
        _color[v2id] = GRAY;
      }
      else if(_color[v2id] == GRAY){
        _f.back_edge(v1,ei);
      }
      else if(_color[v2id] == BLACK){
        _f.ft_edge(v1,ei);
      }
      else cout<<"ERROR: colors in BFS functor"<<endl;
    }
      }
      _color[v1id] = BLACK;
      _f.finish_vertex(v1);
    } else {   
      cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
    }
    q.pop_front();
  }
  return 1;
}

/**BFS functor; Apply visitor _f to all the nodes in _g accesible
 * from _startV(user data) in a BFS manner
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startV start vertex for traversal(user data)
 * @return ERROR if something wrong happened; else OK
 */
template<class GRAPH>
int BFS_functor (GRAPH& _g , visitor_base<GRAPH>& _f, typename GRAPH::VERTEX_TYPE& _startV) {
  typename GRAPH::CVI cv1;
  if ( _g.IsVertex(_startV,&cv1) ) {
    map<VID, int> _color;
    int temp = _functor_BFS(_g, _f, cv1->vid, _color);
    _color.clear();
    return temp;
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return ERROR;
  }
}
/**BFS functor; Apply visitor _f to all the nodes in _g accesible
 * from _startVid in a BFS manner
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startVid start vertex for traversal
 * @return ERROR if something wrong happened
 */
template<class GRAPH>
int BFS_functor (GRAPH& _g , visitor_base<GRAPH>& _f, VID _startVid) {
  map<VID, int> _color;
  int temp = _functor_BFS(_g, _f, _startVid, _color);
  _color.clear();
  return temp;
}


/**BFS functor; Apply visitor _f to all the nodes in _g 
 * in a BFS manner
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 * @return ERROR if something wrong happened
 */
template<class GRAPH>
int BFS_functor (GRAPH& _g , visitor_base<GRAPH>& _f) {
  map<VID, int> _color;
  int temp;
  //for all vertices check if it was touched
  for(typename GRAPH::VI vi = _g.begin(); vi != _g.end(); vi++) {    
    if(_color[vi->vid] == WHITE)
      temp = _functor_BFS(_g, _f, vi->vid, _color);
    if(temp == ERROR) return ERROR;
  }
  _color.clear();
  return temp;
}


/*
DFS_functor internal function
*/
template <class GRAPH>
int _recursive_DFS(GRAPH& _g, visitor_base<GRAPH>& _f, map<VID,int>& _color, VID _v1id){

  VID v2id;
  typename GRAPH::VI  v1,v2;
  typename GRAPH::EI ei;
  
  if ( !_g.IsVertex(_v1id, &v1)){
    cout << "\nIn GraphDFS: vid=" << _v1id << " not in graph";
    return ERROR;
  }
  _f.vertex(v1);
  
  ei = v1->edgelist.begin(); 
  while ( ei != v1->edgelist.end() ) {
    v2id = ei->vertex2id;
    if ( _g.IsVertex(v2id, &v2) ) {
      if( _color[v2id]==WHITE ) {
    _f.tree_edge(v1,ei);
    _color[v2id]=GRAY;
    int temp =  _recursive_DFS(_g, _f, _color, v2id);
    if(temp == ERROR) return ERROR;
      }
      else if(_color[v2id] == GRAY){
    _f.back_edge(v1,ei);
      }
      else if(_color[v2id] == BLACK){
    _f.ft_edge(v1,ei);
      }
      else cout<<"ERROR: colors in DFS functor"<<endl;
    } else cout << "\nIn GraphDFS: vid=" << v2id << " not in graph";
    ei++;
  }  
  _color[_v1id] = BLACK;
  _f.finish_vertex(v1);
  return OK;
}

/**DFS functor; Apply visitor _f to all the nodes in _g accesible
 * from _startV(user data) in a BFS manner
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startV start vertex for traversal(user data)
 * @return ERROR if something wrong happened
 */
template<class GRAPH>
int DFS_functor (GRAPH& _g , visitor_base<GRAPH>& _f, typename GRAPH::VERTEX_TYPE& _startV) {
  typename GRAPH::CVI cv1;
  if ( _g.IsVertex(_startV,&cv1) ) {
    map<VID, int> _color;
    _color[cv1->vid] =  GRAY;
    int temp = _recursive_DFS(_g, _f, _color, cv1->vid);
    _color.clear();
    return temp;
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}


/**DFS functor; Apply visitor _f to all the nodes in _g accesible
 * from _startVid in a BFS manner
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startVid start vertex for traversal
 * @return ERROR if something wrong happened
 */
template<class GRAPH>
int DFS_functor (GRAPH& _g , visitor_base<GRAPH>& _f, VID _startVid) {
  map<VID, int> _color;
  _color[_startVid] =  GRAY;
  int temp = _recursive_DFS(_g, _f, _color, _startVid);
  _color.clear();
  return temp;
}


/**DFS functor; Apply visitor _f to all the nodes in _g
 * in a DFS manner; it builds a forest of DFS trees
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 * @return ERROR if something wrong happened
 */
template<class GRAPH>
int DFS_functor (GRAPH& _g , visitor_base<GRAPH>& _f) {
  map<VID, int> _color;
  int temp;
  //for all vertices check if it was touched
  for(typename GRAPH::VI vi = _g.begin(); vi != _g.end(); vi++) {    
    if(_color[vi->vid] == WHITE){
      _color[vi->vid] = GRAY;
      temp = _recursive_DFS(_g, _f, _color,vi->vid);
      if(temp == ERROR) return ERROR;
    }
  }
  _color.clear();
  return temp;
}


//////////////////////////////////////////////////
// early quit BFS
//////////////////////////////////////////////////

#define EARLY_QUIT -2

template<class GRAPH>
int _functor_BFS_EQ (GRAPH& _g , 
             visitor_base<GRAPH>& _f, 
             VID _startVid,  
             map<VID, int>& _color) {
   
  list<VID> q; 
  typename GRAPH::VI v1,v2;
  typename GRAPH::EI ei;

  VID v1id, v2id; 


  if ( _g.IsVertex(_startVid,&v1) ) {
    q.push_back(_startVid);
    if(_f.vertex(v1) == EARLY_QUIT) return EARLY_QUIT;
    _color[_startVid] = GRAY;
  } else {
    cout << "\nIn GraphBFS: root vid=" << _startVid << " not in graph";
    return -1; 
  }
  while ( !q.empty() ) {
    v1id = q.front();
    if ( _g.IsVertex(v1id,&v1) ){
      for (ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
    v2id = ei->vertex2id;
    if ( _g.IsVertex(v2id,&v2)) { 
      if(_color[v2id] == WHITE){
        if(_f.vertex(v2) == EARLY_QUIT) return EARLY_QUIT;
        if(_f.tree_edge(v1,ei) == EARLY_QUIT) return EARLY_QUIT;
        q.push_back(v2id);
        _color[v2id] = GRAY;
      }
      else if(_color[v2id] == GRAY){
        if(_f.back_edge(v1,ei) == EARLY_QUIT) return EARLY_QUIT;
      }
      else if(_color[v2id] == BLACK){
        if(_f.ft_edge(v1,ei) == EARLY_QUIT) return EARLY_QUIT;
      }
      else cout<<"ERROR: BFS EQ colors"<<endl;
    } else cout << "\nIn GraphBFS: OOPS! vertex=" << v2id << " not in graph";
      }
      _color[v1id] = BLACK;
      if(_f.finish_vertex(v1) == EARLY_QUIT) return EARLY_QUIT;
    } else {   
      cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
    }
    q.pop_front();
  }
  _color.clear();
  return OK;
}


/**BFS functor EARLY QUIT; Apply visitor _f to all the nodes in _g accesible
 * from _startV in a BFS manner; If one of the visitor methods returns EARLY_QUIT
 * the BFS search will be cancelled(EARLY QUIT)
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startV start vertex for traversal
 *@return the success of the operation
 */
template<class GRAPH>
int BFS_functor_EQ (GRAPH& _g , visitor_base<GRAPH>& _f, typename GRAPH::VERTEX_TYPE& _startV) {
  typename GRAPH::CVI cv1;
  if ( _g.IsVertex(_startV,&cv1) ) {
    map<VID, int> _color;
    int temp = _functor_BFS_EQ(_g, _f, cv1->vid,_color);
    _color.clear();
    return temp;
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}

/**BFS functor EARLY QUIT; Apply visitor _f to all the nodes in _g accesible
 * from _startVid in a BFS manner; If one of the visitor methods returns EARLY_QUIT
 * the BFS search will be cancelled(EARLY QUIT)
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startVid start vertex for traversal
 *@return the success of the operation
 */
template<class GRAPH>
int BFS_functor_EQ (GRAPH& _g , visitor_base<GRAPH>& _f, VID _startVid) {
  map<VID, int> _color;
  int temp = _functor_BFS_EQ(_g, _f, _startVid, _color);
  _color.clear();
  return temp;
}


//////////////////////////////////////////////////
// early quit DFS
//////////////////////////////////////////////////

template <class GRAPH>
int _recursive_DFS_EQ(GRAPH& _g, visitor_base<GRAPH>& _f, map<VID,int>& _color, VID _v1id){
  
  VID v2id;
  typename GRAPH::VI  v1,v2;
  typename GRAPH::EI ei;
  
  if ( !_g.IsVertex(_v1id, &v1)){
    //cout << "\nIn GraphDFS: vid=" << _v1id << " not in graph";
    return ERROR;//!!!!! this has to be different than early quit;  ERROR
  }
  if(_f.vertex(v1) == EARLY_QUIT) return EARLY_QUIT;

  ei = v1->edgelist.begin(); 
  while ( ei != v1->edgelist.end() ) {
    v2id = ei->vertex2id;
    if ( _g.IsVertex(v2id, &v2) ) {
      if( _color[v2id]==WHITE ) {
    if(_f.tree_edge(v1,ei)==EARLY_QUIT) return EARLY_QUIT;
    _color[v2id]=GRAY;
    int temp =  _recursive_DFS_EQ(_g, _f, _color, v2id);
    if(temp == EARLY_QUIT) return EARLY_QUIT;
      } 
      else if(_color[v2id] == GRAY){
    if(_f.back_edge(v1,ei) == EARLY_QUIT) return EARLY_QUIT;
      }
      else if(_color[v2id] == BLACK){
    if(_f.ft_edge(v1,ei) == EARLY_QUIT) return EARLY_QUIT;
      }
      else cout<<"ERROR: DFS EQ colors"<<endl;
    } else {
      //cout << "\nIn GraphDFS: vid=" << v2id << " not in graph";
    }
    ei++;
  }  
  _color[_v1id] = BLACK;
  if(_f.finish_vertex(v1) == EARLY_QUIT) return EARLY_QUIT;
  return OK;
}

/**DFS functor EARLY QUIT; Apply visitor _f to all the nodes in _g accesible
 * from _startV in a DFS manner; If one of the visitor methods returns EARLY_QUIT
 * the DFS search will be cancelled(EARLY QUIT)
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startV start vertex for traversal
 * @return the success of the operation
 */
template<class GRAPH, class VERTEX>
int DFS_functor_EQ (GRAPH& _g , visitor_base<GRAPH>& _f, VERTEX& _startV) {
  typename GRAPH::CVI cv1;
  if ( _g.IsVertex(_startV,&cv1) ) {
    map<VID, int> _color;
    _color[cv1->vid] =  GRAY;
    int temp = _recursive_DFS_EQ(_g, _f, _color, cv1->vid);
    _color.clear();
    return temp;
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}

/**DFS functor EARLY QUIT; Apply visitor _f to all the nodes in _g accesible
 * from _startVid in a DFS manner; If one of the visitor methods returns EARLY_QUIT
 * the DFS search will be cancelled(EARLY QUIT)
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 *@param _startVid start vertex for traversal
 * @return the success of the operation
 */
template<class GRAPH>
int DFS_functor_EQ (GRAPH& _g , visitor_base<GRAPH>& _f, VID _startVid) {
  map<VID, int> _color;
  _color[_startVid] =  GRAY;
  int temp  = _recursive_DFS_EQ(_g, _f, _color, _startVid);
  _color.clear();
  return temp;
}

/**DFS functor EARLY QUIT; Apply visitor _f to all the nodes in _g
 * in a DFS manner; If one of the visitor methods returns EARLY_QUIT
 * the DFS search will be cancelled(EARLY QUIT)
 *@param _g the graph for which we apply DFS
 *@param _f the visitor that will be used for traversal
 * @return the success of the operation
 */
template<class GRAPH>
int DFS_functor_EQ (GRAPH& _g , visitor_base<GRAPH>& _f) {
  map<VID, int> _color;
  int temp;
  //for all vertices check if it was touched
  for(typename GRAPH::VI vi = _g.begin(); vi != _g.end(); vi++) {    
    if(_color[vi->vid] == WHITE){
      _color[vi->vid] = GRAY;
      temp = _recursive_DFS_EQ(_g, _f, _color,vi->vid);
      if(temp == EARLY_QUIT) {_color.clear();return EARLY_QUIT;}
      if(temp == ERROR) {_color.clear();return ERROR;}
    }
  }
  _color.clear();
  return temp;
}

//////////////////////////////////////////////////
//original Graph.h DFS ;; returns a Graph; do not use it anymore it will be taken out
//////////////////////////////////////////////////
template<class DGRAPH, class GRAPH>
int true_DFS (GRAPH& g, DGRAPH& dfstree, VID & vid, dfsinfo & dfs) {

  //core DFS algorithm, start w/ vertex vid
  //return a dfstree started at vid
  //also return dfsinfo
  
  // typename GRAPH::CVI cv1,cv2;
  typename GRAPH::VI  v1;
  VID v1id=0,v2id=0;  //v1id=parent(vid), v2id=adj(vid)
  int k=1;
  static int ftime=0; //static finish time,  the finish time for different 
  //trees is in consecutive order
  dfstree.EraseGraph();
  
  dfs.color[vid] = 1;
  dfs.vnode[k] = vid;  
  if ( !g.IsVertex(vid, &v1)) 
    cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
  dfstree.AddVertex(v1->data,vid);
  while( k > 0 ) {
    vid = dfs.vnode[k];
    if ( !g.IsVertex(vid, &v1)) 
      cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
    //v1 = const_cast<typename GRAPH::VI>(cv1);
    typename GRAPH::CEI e = v1->edgelist.begin(); 
    while ( e < v1->edgelist.end() ) {
      v2id = e->vertex2id;
      if( !dfs.color[v2id]) {
    v1id = vid; //parent
    vid = v2id; //child
    dfs.color[vid] = 1;
    dfs.vnode[++k] = vid;
    if ( g.IsVertex(vid, &v1) ) {
      //v1 = const_cast<typename GRAPH::VI>(cv2); //set current v as child
      dfstree.AddVertex(v1->data,vid);
      dfstree.AddEdge(v1id,vid,e->weight);
      e = v1->edgelist.begin();
    } else cout << "\nIn GraphDFS: vid=" << vid << " not in graph";
      }  
      else if( dfs.color[v2id] == 1) {    //gray
    pair<VID,VID> backedge(v2id,vid);
    dfs.backedge_vector.push_back(backedge); //record back edge
    e++;
    //cout<<"backedge:"<<v2id<<" " << vid<<endl;
      }
      else e++;
    }
    dfs.finish_time[dfs.vnode[k]]=++ftime;
    dfs.color[dfs.vnode[k]] = 2; //black
    k--; //back to parent
  }
  return dfstree.GetVertexCount();
}

/**
*Visitor used to build the DFS/BFS tree;
*g is the original graph and bfstree is the BFS/DFS tree that will
*built for the current traversal;
*/
template <class GRAPH,class DGRAPH>
class _visitor_BFSDFS: public visitor_base<GRAPH>{
  GRAPH* g;
  DGRAPH* bfstree;
 public:  
  _visitor_BFSDFS(GRAPH* _g, DGRAPH* _bfst){
    bfstree = _bfst;
    bfstree->EraseGraph();
    g = _g;
  };
  inline int vertex (typename GRAPH::VI vi){
    if(!bfstree->IsVertex(vi->vid))
      bfstree->AddVertex(vi->data,vi->vid); 
    return 1;
  }
  inline int tree_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    VID v2id = ei->vertex2id;
    typename GRAPH::VI v1;
    if(!bfstree->IsVertex(v2id) && g->IsVertex(v2id,&v1)){
      bfstree->AddVertex(v1->data,v2id); 
    }
    if ( bfstree->AddEdge(vi->vid,v2id,ei->weight) != OK) {
      cout << "\nIn GraphBFSDFS: OOPS! edge not added right...";
    }
    return 1;
  }
};



template<class GRAPH>
void aux_DFS (GRAPH& g, dfsinfo& dfs) {

  //driver, to find all backedges, and topological order of vertices
  typedef Graph<DG<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>, 
    MG<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>, 
    WG<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>, 
    typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE> DGRAPH;

  typename GRAPH::VI  v1;
  VID vid;  
  
  for (v1 = g.begin(); v1 < g.end(); v1++) {
    vid = v1->vid;
    if( dfs.color[vid] == 0 ){
      DGRAPH temp_g;
      true_DFS(g, temp_g, vid, dfs);
    }
  }   
}

/**
* old DFS it will be replaced by DFS_VIS(the one that is using the visitor)
*/
template<class DGRAPH, class GRAPH>
int DFS_OLD (GRAPH& g, DGRAPH& dfstree, typename GRAPH::VERTEX_TYPE& _startV)  {
  typename GRAPH::CVI cv1;
  if ( g.IsVertex(_startV,&cv1) ) {
    return DFS(g, dfstree, cv1->vid);
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0; 
  }
}

/**
* old DFS it will be replaced by DFS_VIS(the one that is using the visitor)
*/
template<class DGRAPH, class GRAPH>
int DFS_OLD (GRAPH& g, DGRAPH& dfstree, VID vid) {
  //To find one dfs tree w/ start vertex vid 
  dfsinfo dfs(g.GetVertexCount());
  return true_DFS(g, dfstree, vid, dfs);
}

/**
* DFS; it builds the DFS tree for the graph _g starting with vertex _startV; 
*@return The succes of the opeartion or not.
*/
template<class GRAPH, class DGRAPH>
int DFS (GRAPH& g, DGRAPH& dfstree, typename GRAPH::VERTEX_TYPE& _startV) {
  typename GRAPH::CVI cv1;
  if ( g.IsVertex(_startV,&cv1) ) {
    _visitor_BFSDFS<GRAPH,DGRAPH> _vis(&g,&dfstree);
    return DFS_functor(g,_vis,cv1->vid);
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}

/**
* DFS; it builds the DFS tree for the graph _g starting with vertex _startV; 
*@return The succes of the opeartion or not.
*/
template<class DGRAPH, class GRAPH>
int DFS (GRAPH& g ,DGRAPH& dfstree, VID _startVid) {
  _visitor_BFSDFS<GRAPH,DGRAPH> _vis(&g,&dfstree);
  return DFS_functor(g,_vis,_startVid); 
}


template <class GRAPH>
class _visitor_dfsvid: public visitor_base<GRAPH>{
  vector<VID>* ccverts;
public:  
  _visitor_dfsvid(GRAPH& g, vector<VID>* _v){
   ccverts = _v;
  };
  inline  int vertex(typename GRAPH::VI vert) {
    ccverts->push_back(vert->vid);
    return 1;
  }
};

/**
* DFSVID; it builds the vector of VIDs as seen by DFS traversal;
*@param _g the graph for which we apply DFS
*@param _startV(user data) start vertex for traversal
*@param _V the vector where the data will be stored;
*/
template<class GRAPH, class VERTEX>
int DFSVID (GRAPH& _g, VERTEX& _startV, vector<VID>& _V) {
  typename GRAPH::CVI cv1;
  if ( _g.IsVertex(_startV,&cv1) ) {
    _V.clear();
    _visitor_dfsvid<GRAPH> _vis(_g, &_V);
    DFS_functor(_g ,_vis , cv1->vid);
    return _V.size();
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}

/**
* DFSVID; it builds the vector of VIDs as seen by DFS traversal;
*@param _g the graph for which we apply DFS
*@param _startVID start vertex for traversal
*@param _V the vector where the data will be stored;
*/
template<class GRAPH>
int  DFSVID (GRAPH& _g, VID& _startVID, vector<VID>& _V) {
  _V.clear();
  _visitor_dfsvid<GRAPH> _vis(_g, &_V);
  DFS_functor(_g ,_vis , _startVID);
  return _V.size();
}


/**
* DFS ; builds up the DFS forest(old style)
*@param _g the graph for which we apply DFS
*@param _V the vector of traversal trees(Directed Graphs) where the data will be stored;
*/
template<class DGRAPH, class GRAPH>
int DFS_OLD (GRAPH& g, vector<DGRAPH>& _V) {
  //To find all dfs trees
  
  typename GRAPH::VI  v1;
  VID vid;
  int count=0;
  dfsinfo dfs(g.GetVertexCount());
  _V.clear();

  for (v1 = g.begin(); v1 < g.end(); v1++) {
    vid = v1->vid;
    if( dfs.color[vid] == 0 ){
      _V.push_back();//add one more graph
      true_DFS(g, _V[count], vid, dfs);
      count++;
    }
  }
  return count;
}




template <class GRAPH,class DGRAPH>
class _visitor_DFSFOREST: public visitor_base<GRAPH>{
  GRAPH* g;
  typename vector<DGRAPH>::iterator bfstree;
 public:  
  _visitor_DFSFOREST(GRAPH* _g, typename vector<DGRAPH>::iterator _bfst){
    bfstree = _bfst;
    bfstree->EraseGraph();
    g = _g;
  };
  inline int vertex (typename GRAPH::VI vi){
    if(!bfstree->IsVertex(vi->vid))
      bfstree->AddVertex(vi->data,vi->vid); 
    return 1;
  }
  inline int tree_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    VID v2id = ei->vertex2id;
    typename GRAPH::VI v1;
    if(!bfstree->IsVertex(v2id) && g->IsVertex(v2id,&v1)){
      bfstree->AddVertex(v1->data,v2id); 
    }
    if ( bfstree->AddEdge(vi->vid,v2id,ei->weight) != OK) {
      cout << "\nIn GraphBFSDFS: OOPS! edge not added right...";
    }
    return 1;
  }
};

/**
* DFS ; builds up the DFS forest
*@param _g the graph for which we apply DFS
*@param _V the vector of traversal trees(Directed Graphs) where the data will be stored;
*/
template<class DGRAPH, class GRAPH>
int DFS (GRAPH& _g , vector<DGRAPH>& _V) {
  map<VID, int> _color;
  int temp;
  //for all vertices check if it was touched
  for(typename GRAPH::VI vi = _g.begin(); vi != _g.end(); vi++) {    
    if(_color[vi->vid] == WHITE){
      _color[vi->vid] = GRAY;
    DGRAPH dgr;
      _V.push_back(dgr);//add one more graph
      _visitor_DFSFOREST<GRAPH,DGRAPH> _vis(&_g , _V.end()-1);
      temp = _recursive_DFS(_g, _vis, _color,vi->vid);
      if(temp == ERROR) {_color.clear();return ERROR;}
    }
  }
  _color.clear();
  return temp;
}


/****************
ISCycle functions
  - the old IsCycle
  - and the one with visitor
*****************/

template <class GRAPH>
class _visitor_cycle: public visitor_base<GRAPH>{
  public:  
  _visitor_cycle(GRAPH& g){
  };
  inline int back_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    return EARLY_QUIT;
  }  
};

/**Check if there is any cycle in this graph.
 *@return true if there is cycle(s) in this graph, otherwise 
 *@see dfs.backedge_vector and aux_DFS
 */
template<class GRAPH>
bool IsCycle (GRAPH& _g) {
  _visitor_cycle<GRAPH> _vis(_g);
  int temp = DFS_functor_EQ(_g ,_vis);  
  if(temp == EARLY_QUIT) return true;
  else return false;
}

//old IsCycle; it will be removed
template<class GRAPH>
bool IsCycle_OLD (GRAPH& g) {
  dfsinfo dfs(g.GetVertexCount());
  aux_DFS(g, dfs);
  if( dfs.backedge_vector.empty() ) return false;
  else return true;
}

/********************
GetBackEdge functions
*********************/
template <class GRAPH>
class _visitor_back_edges: public visitor_base<GRAPH>{
  vector<pair<VID,VID> >* _back_edges;
  public:  
  _visitor_back_edges(GRAPH& g, vector<pair<VID,VID> >* _v){
    _back_edges = _v;
  };
  inline int back_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    //An is making the pair the other way
    //_back_edges->push_back(pair<VID,VID>(vi->vid,ei->vertex2id));
    _back_edges->push_back(pair<VID,VID>(ei->vertex2id,vi->vid));
    return 1;
  }  
};

/**Get the edges that makes a cycle in the graph(one edge per cycle)
 *@param _g the Graph 
 *@param _v the vector where edges are stored as pair <source, destination>.
 *@return the number of edges
 *@see dfs.backedge_vector and aux_DFS
 */
template<class GRAPH>
int GetBackedge(GRAPH& _g, vector<pair<VID,VID> >& _V) {
  _V.clear();
  _visitor_back_edges<GRAPH> _vis(_g,&_V);
  int temp = DFS_functor(_g ,_vis);  
  return _V.size();
}

//old one
template<class GRAPH>
int GetBackedge_OLD(GRAPH& g, vector<pair<VID,VID> >& _V) {
  dfsinfo dfs(g.GetVertexCount());
  aux_DFS(g, dfs);
  _V = dfs.backedge_vector;
  return _V.size();
}

/***************************************
The old topological sort and the new one
****************************************/ 
template <class T>
struct __FinishLate : public binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.second > y.second; }
};

template<class GRAPH>
int TopologicalSort (GRAPH& g, vector<VID>& _V) {

  //!!!!!!!!!!!!!!! this can be optimized

    int i,n;
    //vector<VID> tps;
    n=g.GetVertexCount();
    vector<pair<VID,int> > tmp;
    tmp.reserve(n);
    _V.clear();
    _V.reserve(n);
    dfsinfo dfs(n);
    aux_DFS(g, dfs);
    
    for(i=0;i<n;i++) {
        pair<VID,int> newpair(i,dfs.finish_time[i]);
        tmp.push_back(newpair);
    }   

#ifdef _STLP_STD
    stable_sort(tmp.begin(),tmp.end(),__FinishLate<pair<VID,int> >());
#else 
    stable_sort(tmp.begin(),tmp.end(),__FinishLate<pair<VID,int> >());
#endif

    for(i=0; i<n;i++) {
      _V.push_back(tmp[i].first);
#if DEBUG
    cout<<"\nTopological Sort results: "<<endl;
        cout<<tmp[i].first<<" ";
#endif
    }
    return _V.size();            
}

template <class GRAPH>
class _visitor_topsort: public visitor_base<GRAPH>{
  vector<VID>* ccverts;
public:  
  _visitor_topsort(GRAPH& g, vector<VID>* _v){
   ccverts = _v;
  };
  inline  int finish_vertex(typename GRAPH::VI vert) {
    ccverts->push_back(vert->vid);
    return 1;
  }
};


 /**Topological sort,
  *@param _g the graph
  *@param _V a list of vids sorted in the topological order
  *@return the number of elements stored in _V
  */
template<class GRAPH>
int  TopologicalSort_VIS (GRAPH& _g, vector<VID>& _V) {
  _visitor_topsort<GRAPH> _vis(_g, &_V);
  DFS_functor(_g ,_vis);
  reverse(_V.begin(), _V.end());
  return _V.size();
}

//*************************************************************** 
//  BREADTH-FIRST-SEARCH ALGORITHMS
//*************************************************************** 

template<class DGRAPH, class GRAPH>
int true_BFS (GRAPH& g ,DGRAPH& bfstree, VID _startVid) {
   
  list<VID> q; 
  //typename GRAPH::CVI cv1,cv2;
  typename GRAPH::VI v1,v2;
  VID v1id, v2id; 
  bfstree.EraseGraph();
  
  if ( g.IsVertex(_startVid,&v1) ) {
    q.push_back(_startVid);
    //v1 = const_cast<typename GRAPH::VI>(cv1);
    bfstree.AddVertex(v1->data,_startVid); 
  } else {
    cout << "\nIn GraphBFS: root vid=" << _startVid << " not in graph";
    return 0; 
  }
    
  while ( !q.empty() ) {
    v1id = q.front();
    if ( g.IsVertex(v1id,&v1) ) {
      for (typename GRAPH::CEI e = v1->edgelist.begin(); e < v1->edgelist.end(); e++) {
    v2id = e->vertex2id;
    if ( !bfstree.IsVertex(v2id) && g.IsVertex(v2id,&v2) ) { 
      q.push_back(v2id);
      //v2 = const_cast<typename GRAPH::VI>(cv2);
      bfstree.AddVertex(v2->data,v2id); 
      if ( bfstree.AddEdge(v1id,v2id,e->weight) != OK) {
        cout << "\nIn GraphBFS: OOPS! edge not added right...";
      }
    }
      }
    } else {   
      cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
    }
    q.pop_front();
  }
  bfstree.setVertIDs(g.getVertIDs()); // set the same vert ID as in graph
  return bfstree.GetVertexCount();
}

template<class DGRAPH, class GRAPH>
int BFS_OLD (GRAPH& g, DGRAPH& bfstree, typename GRAPH::VERTEX_TYPE& _startV) {
  typename GRAPH::CVI cv1;
  if ( g.IsVertex(_startV,&cv1) ) {
    return true_BFS(g, bfstree, cv1->vid);
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}
template<class DGRAPH, class GRAPH>
int BFS_OLD (GRAPH& g ,DGRAPH& bfstree, VID _startVid) {
  return true_BFS(g, bfstree, _startVid);
}

/**Breadth First Search,
 *Starting to build tree(s) from given VID vertex.
 *A Graph that contains BFS tree will be returned.
  *@param _g the graph
  *@param bfstree the tree that will be built by algorithm
  *@param _startV the start vertex(user data)
 *@return the success of the operation will
 *be returned. If something wrong during processing, error message will 
 *be output to standard output.
 */
template<class GRAPH, class DGRAPH>
int BFS (GRAPH& g, DGRAPH& bfstree, typename GRAPH::VERTEX_TYPE& _startV) {
  typename GRAPH::CVI cv1;
  if ( g.IsVertex(_startV,&cv1) ) {
    bfstree.EraseGraph();
    _visitor_BFSDFS<GRAPH,DGRAPH> _vis(&g,&bfstree);
    return BFS_functor(g,_vis,cv1->vid);
    //return true_BFS(g, bfstree, cv1->vid);
  } else {
    cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
    return 0;
  }
}

/**Breadth First Search,
  *Starting to build tree(s) from given VID vertex.
  *A Graph that contains BFS tree(s) will be returned.
  *@param _g the graph
  *@param bfstree the tree that will be built by algorithm
  *@param _startVid the start vertex id
 *@return the success of the operation will
 *be returned. If something wrong during processing, error message will 
 *be output to standard output.
 */
template<class DGRAPH, class GRAPH>
int BFS (GRAPH& g ,DGRAPH& bfstree, VID _startVid) {
  bfstree.EraseGraph();
  _visitor_BFSDFS<GRAPH,DGRAPH> _vis(&g,&bfstree);
  return BFS_functor(g,_vis,_startVid); 
}



template <class GRAPH>
class _visitor_bfsvid: public visitor_base<GRAPH>{
  vector<VID>* ccverts;
public:  
  _visitor_bfsvid(GRAPH& g, vector<VID>* _v){
   ccverts = _v;
  };
  inline  int vertex(typename GRAPH::VI v) {
    ccverts->push_back(v->vid);
    return 1;
  }
};

//TESTED
/**Breadth First Search,
  *VIDs will be returned instead of a tree(BFS); the VIDs are sorted 
  *@param _g the graph
  *@param _startV the start vertex(user data)
  *@param _V the vector where the VIDs will be stores
  *@return the number of elements pushed in _V
  *@see BFS, GetCC
 */
template<class GRAPH, class VERTEX>
int BFSVID (GRAPH& _g, VERTEX& _startV, vector<VID>& _V) {
 typename GRAPH::CVI cv1;
 if ( _g.IsVertex(_startV,&cv1) ) {
   _V.clear();
   _visitor_bfsvid<GRAPH> _vis(_g, &_V);
   BFS_functor(_g ,_vis , cv1->vid);
   
   stable_sort(_V.begin(),_V.end());
   return _V.size();
 } else {
   cout << "\nIn GraphBFS: root vertex=" << _startV  << " not in graph";
   return 0;
 }
}

//TESTED
/**Breadth First Search,
  *VIDs will be returned instead of a tree(BFS); the VIDs are sorted 
  *@param _g the graph
  *@param _startVID the start vertex id
  *@param _V the vector where the VIDs will be stores
  *@return the number of elements pushed in _V
  *@see BFS, GetCC
 */
template<class GRAPH>
int BFSVID (GRAPH& _g, VID& _startVID, vector<VID>& _V) {
  _V.clear();
  _visitor_bfsvid<GRAPH> _vis(_g, &_V);
  BFS_functor(_g ,_vis , _startVID);
  stable_sort(_V.begin(),_V.end());
  return _V.size();
}

//==============================
//  FindPathBFS (for any graph)
//  -- returns BFS path between 2 specified vertices 
//==============================
//TESTED
/**Find BFS path between 2 specified vertices.
 *@param _g the graph
 *@param _startV the start vertex(user data)
 *@param _endV destination vertex(user data)
 *@param _path the vector where the path will be stored.
 *@return the size of the path
 *@note works only for weighted graphs
 */
template<class GRAPH>
int FindPathBFS (GRAPH& _g, 
         typename GRAPH::VERTEX_TYPE& _startV, 
         typename GRAPH::VERTEX_TYPE& _endV,
         vector< pair<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE> >& _path )  {
    
    typename GRAPH::CVI cv1,cv2;
    if ( _g.IsVertex(_startV,&cv1) && _g.IsVertex(_endV,&cv2) ){
        return FindPathBFS(_g, cv1->vid,cv2->vid,_path);
    } else {
        cout << "\nIn FindPathBFS: start or goal vertex (";
        cout << _startV << ", " << _endV << ") not in graph";
        return 0;
    }
}

//TESTED
/**Find BFS path between 2 specified vertices.
 *@param _g the graph
 *@param _startVid the start vertex
 *@param _endVid destination vertex
 *@param _path the vector where the path will be stored.
 *@return the size of the path
 *@see FindPathBFS(VID,VID) and IsVertex(VERTEX&)
 */
template<class GRAPH>
int FindPathBFS (GRAPH& g, 
         VID _startVid, 
         VID _endVid,
         vector< pair<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE> >& _path ) {

  GRAPH bfstree;
  list<VID> q;
  //typename GRAPH::CVI cv1,cv2;
  typename GRAPH::VI v1,v2;
  VID v1id, v2id;
  _path.clear();
  if ( g.IsVertex(_startVid,&v1) ) {
     _path.reserve( g.size() );
     q.push_back(_startVid);
     //v1 = const_cast<typename GRAPH::VI>(cv1);
     bfstree.AddVertex(v1->data,_startVid); 
  } else {
     cout << "\nIn FindPathBFS: start vertex (" << _startVid << ") not in graph";
     return 0; 
  }

  while ( !q.empty() && !bfstree.IsVertex(_endVid) ) {
     v1id = q.front();
     if ( g.IsVertex(v1id,&v1) ) {
       for (typename GRAPH::CEI e = v1->edgelist.begin(); e < v1->edgelist.end(); e++) {
         v2id = e->vertex2id;
         if ( !bfstree.IsVertex(v2id) && g.IsVertex(v2id,&v2) ) {
            q.push_back(v2id);
            //v2 = const_cast<typename GRAPH::VI>(cv2);
            bfstree.AddVertex(v2->data,v2id);
            if ( bfstree.AddEdge(v2id,v1id,e->weight) != OK) {
                cout << "\nIn FindPathBFS: OOPS! edge not added right...";
            }
         }
       }
     } else {
       cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
     }
     q.pop_front();
  }

  if ( bfstree.IsVertex(_endVid,&v1) && bfstree.IsVertex(_startVid,&v2) ) {
    typename GRAPH::WEIGHT_TYPE wtt;
    _path.insert( _path.begin(), 
         pair<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>(v1->data,wtt) );
     while ( !(_path.begin()->first ==  v2->data) ) {
        typename GRAPH::CEI e = v1->edgelist.begin();
        v1id = v1->vid;
        v2id = e->vertex2id;
        if ( bfstree.IsVertex(v2id,&v1) ) {
           _path.insert( _path.begin(), pair<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>(v1->data,e->weight) );
           bfstree.DeleteEdge(v1id,v2id);
        } else {
           cout << "In FindPathBFS: hmm....\n";
        }
     }
  }
  return _path.size();
}


//TESTED
/**Find BFS path between 2 specified vertices. Return the path as a vector<VID>
 *@param _g the graph
 *@param _startVid the start vertex
 *@param _endVid destination vertex
 *@param _path the vector where the path will be stored.
 *@return the size of the path
*@see FindPathBFS(VID,VID) and IsVertex(VERTEX&)
 */    
template<class GRAPH>
int FindVIDPathBFS (GRAPH& g, VID _startVid, VID _endVid, vector<VID>& _path) {
 
 typedef Graph<DG<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>, 
    MG<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>, 
    WG<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE>, 
    typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE> DGRAPH;

  DGRAPH bfstree;
  list<VID> q;
  //typename GRAPH::CVI cv1,cv2;
  typename GRAPH::VI v1,v2;
  VID v1id, v2id;
  _path.clear();
  if ( g.IsVertex(_startVid,&v1) ) {
     _path.reserve( g.size() );
     q.push_back(_startVid);
     //v1 = const_cast<typename GRAPH::VI>(cv1);
     bfstree.AddVertex(v1->data,_startVid); 
  } else {
     cout << "\nIn FindPathBFS: start vertex (" << _startVid << ") not in graph";
     return 0; 
  }

  while ( !q.empty() && !bfstree.IsVertex(_endVid) ) {
     v1id = q.front();
     if ( g.IsVertex(v1id,&v1) ) {
       for (typename GRAPH::CEI e = v1->edgelist.begin(); e < v1->edgelist.end(); e++) {
         v2id = e->vertex2id;
         if ( !bfstree.IsVertex(v2id) && g.IsVertex(v2id,&v2) ) {
            q.push_back(v2id);
            //v2 = const_cast<typename GRAPH::VI>(cv2);
            bfstree.AddVertex(v2->data,v2id);
            if ( bfstree.AddEdge(v2id,v1id,e->weight) != OK) {
                cout << "\nIn FindPathBFS: OOPS! edge not added right...";
            }
         }
       }
     } else {
       cout << "\nIn GraphBFS: OOPS! vertex=" << v1id << " not in graph";
     }
     q.pop_front();
  }

  if ( bfstree.IsVertex(_endVid,&v1) && bfstree.IsVertex(_startVid,&v2) ) {
     _path.insert( _path.begin(),v1->vid);
     while ( !(*_path.begin() ==  v2->vid) ) {
        typename GRAPH::CEI e = v1->edgelist.begin();
        v1id = v1->vid;
        v2id = e->vertex2id;
        if ( bfstree.IsVertex(v2id,&v1) ) {
           _path.insert( _path.begin(),v1->vid);
           bfstree.DeleteEdge(v1id,v2id);
        } else {
          // cout << "In FindPathBFS: hmm....\n";
           break;
    }
     }
  }
  return _path.size();
}


template <class GRAPH>
class _visitorCC: public visitor_base<GRAPH>{
  VID v2id;
public:  
  _visitorCC(GRAPH& g, VID _v2){
    v2id = _v2;
  };
  inline int vertex(typename GRAPH::VI v) {
    if(v->vid == v2id) return EARLY_QUIT;
    else return 1;
  }
};
//TESTED
/**Check if vid1 and vid2 are in the same component.
 *@param _g the graph
 *@param _v1id the source vertex  
 *@param _v2id destination vertex  
 *@return true if vid1 and vid2 are in the same component. Otherwise false
 *will be returned.
 *@see BFS(VID)
 */
template <class GRAPH>
bool IsSameCC (GRAPH& _g, VID _v1id, VID _v2id){
  _visitorCC<GRAPH> vis(_g, _v2id); 
  if(BFS_functor_EQ (_g ,vis ,_v1id) == EARLY_QUIT) return true;
  else return false;
}

//TESTED  
/**Check if v1 and v2 are in the same component.
 *Here v1 is the vertex constains user data in first parameter,
 *v2 is the vertex constains user data in second parameter.
 *if there are more than one, then the first will be applied.
 *@param _g the graph
 *@param _v1 the source vertex data 
 *@param _v2 destination vertex data 
 *@return true if v1 and v2 are in the same component. Otherwise false
 *will be returned.
 *@see BFS(VERTEX&)
 */
template <class GRAPH>
bool IsSameCC (GRAPH& _g, typename GRAPH::VERTEX_TYPE& _v1, typename GRAPH::VERTEX_TYPE& _v2){
  typename GRAPH::CVI cv1,cv2;
  vector<char> color(_g.GetVertexCount(),0);
  if ( !_g.IsVertex(_v1,&cv1) ) return false;
  if ( !_g.IsVertex(_v2,&cv2) ) return false;
  
  _visitorCC<GRAPH> vis(_g, cv2->vid);
  if(BFS_functor_EQ (_g ,vis ,cv1->vid) == EARLY_QUIT) return true;
  else return false;
}

template <class GRAPH,class CONTAINER>
class _visitor_GetCC: public visitor_base<GRAPH>{
  CONTAINER* ccverts;
public:  
  _visitor_GetCC(GRAPH& g, CONTAINER* _v){
   ccverts = _v;
  };
  inline  int vertex(typename GRAPH::VI v) {
    ccverts->push_back(v->vid);
    return 1;
  }
};

//TESTED
/**Get a list of vertices which are in the same connected
 *conponent with the specified vertex.
 *@param _g the graph
 *@param _v1 the vertex id that define the connected component
 *@param _ccverts container(vector) where the vertices id will be stored; 
 *@return the number of elements pushed in _ccverts
 *@see BFS(VERTEX&)
 */
template<class GRAPH>
int GetCC ( GRAPH& _g, VID _v1id, vector<VID>& _ccverts) {

  _visitor_GetCC<GRAPH,vector<VID> > _vis(_g, &_ccverts);
  BFS_functor(_g ,_vis , _v1id);
  stable_sort(_ccverts.begin(),_ccverts.end());
  return _ccverts.size();
}

template <class GRAPH,class CONTAINER>
class _visitor_GetCCVERTEX: public visitor_base<GRAPH>{
  CONTAINER* ccverts;
public:  
  _visitor_GetCCVERTEX(GRAPH& g, CONTAINER* _v){
   ccverts = _v;
  };
  inline  int vertex(typename GRAPH::VI v) {
    ccverts->push_back(v->data);
    return 1;
  }
};
//TESTED

/**Get a list of user data of vertices which are in the same connected
 *component with the specified vertex.
 *@param _g the graph
 *@param _v1 the vertex data that define the connected component
 *@param _ccverts container(vector) where the vertices data will be stored; 
 *@return the number of elements pushed in _ccverts
 *@see BFS(VERTEX&)
 */
template<class GRAPH>
int GetCC (GRAPH& _g, typename GRAPH::VERTEX_TYPE& _v1, vector<typename GRAPH::VERTEX_TYPE>& _ccverts) {
  typename GRAPH::CVI cv1;

  if ( !_g.IsVertex(_v1,&cv1) ) return -1;
  _visitor_GetCCVERTEX<GRAPH,vector<typename GRAPH::VERTEX_TYPE> > _vis(_g, &_ccverts);
  BFS_functor(_g ,_vis , cv1->vid); 
  return _ccverts.size();
}

template <class GRAPH,class CONTAINER>
class _visitor_GetCCEdges: public visitor_base<GRAPH>{
  CONTAINER* ccedges;
 public: 
  _visitor_GetCCEdges(GRAPH& g, CONTAINER* _v){
    ccedges = _v;
  };
  inline int tree_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    pair<VID,VID> nextedge(vi->vid,ei->vertex2id);
    ccedges->push_back(nextedge);
    return 1;
  }
  inline int back_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    
    pair<VID,VID> nextedge(vi->vid,ei->vertex2id);
    ccedges->push_back(nextedge);
    return 1;
  } 
  inline int ft_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    pair<VID,VID> nextedge(vi->vid,ei->vertex2id);
    ccedges->push_back(nextedge);
    return 1;
  }
};

template <class GRAPH,class CONTAINER>
class _visitor_GetCCEdgesWt: public visitor_base<GRAPH>{
  CONTAINER* ccedges;
 public:  
  _visitor_GetCCEdgesWt(GRAPH g, CONTAINER* _v){
   ccedges = _v;
  };
  inline int tree_edge (typename GRAPH::VI vi, typename GRAPH::EI ei)  {
    pair<VID,VID> nextedge(vi->vid,ei->vertex2id);
    pair<pair<VID,VID>,typename GRAPH::WEIGHT_TYPE> 
      nextedgewt(nextedge,ei->weight);
    ccedges->push_back(nextedgewt);
    return 1;
  }
  inline int back_edge (typename GRAPH::VI vi, typename GRAPH::EI ei)  {
    pair<VID,VID> nextedge(vi->vid,ei->vertex2id);
    pair<pair<VID,VID>,typename GRAPH::WEIGHT_TYPE> nextedgewt(nextedge,ei->weight);
    ccedges->push_back(nextedgewt);
    return 1;
  }
  inline int ft_edge (typename GRAPH::VI vi, typename GRAPH::EI ei){
    pair<VID,VID> nextedge(vi->vid,ei->vertex2id);
    pair<pair<VID,VID>,typename GRAPH::WEIGHT_TYPE> nextedgewt(nextedge,ei->weight);
    ccedges->push_back(nextedgewt);
    return 1;
  }
};

//bool _compare_pair_vid(pair<VID,VID> a, pair<VID,VID> b) {    return a.first< b.first;    }

template<class PVID>
class _compare_pair_vid{    
 public:
  bool operator () (PVID a, PVID b){        
    return a.first < b.first;   
  }
};
//TESTED
/**Get edges in connected component defined this specified vertex(unweighted).
 *@param _g the graph
 *@param ccedges container(vector) where the edges will be stored; 
 *@param _v1 the vertex id that define the connected component
 *@note This returned edge list might contains duplicate edges. this
 *is when vid1->vid2 was in the list, vid2->vid1 might also be in the list.
 *@return the number of elements pushed in _ccedges
 *@see BFS(VID) and GetIncidentEdges
 */
template<class GRAPH>
int GetCCEdges (GRAPH& _g, vector<pair<VID,VID> >& _ccedges,  VID _v1id) {
  _visitor_GetCCEdges<GRAPH,vector<pair<VID,VID> > > _vis(_g, &_ccedges);
  BFS_functor(_g ,_vis , _v1id); 
  _compare_pair_vid< pair<VID,VID>  > _compare;
  stable_sort(_ccedges.begin(),_ccedges.end(),_compare);
  return _ccedges.size();
}

template<class PVID_WT>
class _compare_pair_vid_weight
{   public:
    bool operator () (PVID_WT a, PVID_WT b) {       return a.first.first < b.first.first;   }

};

//TESTED
/**Get edges in connected conponent defined this specified vertex(WEIGHTED).
 *@param _g the graph
 *@param _ccedges container(vector) where the edges will be stored; 
 *@param _v1 the vertex id that define the connected component
 *@note This returned edge list might contains duplicate edges. this
 *is when vid1->vid2 was in the list, vid2->vid1 might also be in the list.
 *@return the number of elements pushed in _ccedges
 *@see BFS(VID) and GetIncidentEdges
 */
/*
template<class GRAPH>
int GetCCEdges (GRAPH& _g, vector<pair<pair<VID,VID>,typename GRAPH::WEIGHT_TYPE> >& _ccedges,  VID _v1id) {
  _visitor_GetCCEdgesWt<GRAPH,vector<pair<pair<VID,VID>,typename GRAPH::WEIGHT_TYPE> > > _vis(_g, &_ccedges);
  BFS_functor(_g ,_vis , _v1id); 
  _compare_pair_vid_weight<  pair<pair<VID,VID>,typename GRAPH::WEIGHT_TYPE>  > _compare;
  stable_sort(_ccedges.begin(),_ccedges.end(),_compare );
  return _ccedges.size();
}
*/

//TESTED 
/**Get edges in connected conponent defined this specified vertex.
 *this specified vertex is defiend by given user data reference.
 *@param _g the graph
 *@param _ccedges container(vector) where the edges will be stored; this can be 
 *vector<pair<VID,VID>> for unweighted or vector<pair<pair<VID,VID>,Weihght> for weighted Graphs
 *@param _v1 the vertex that define the connected component(user data)
 *@note This returned edge list might contains duplicate edges. this
 *is when vid1->vid2 was in the list, vid2->vid1 might also be in the list.
 *
 *@return the number of elements pushed in _ccedges
 *@see GetCCEdges(VID) and GetVID(VERTEX&)
 */
template<class GRAPH, class CONTAINER>
int GetCCEdges (GRAPH& _g, 
        CONTAINER& _ccedges, 
        typename GRAPH::VERTEX_TYPE&  _v1) {
    return GetCCEdges (_g, _ccedges, _g.GetVID(_v1) );
}


//TESTED
/**Get user data associated with edges in connected conponent 
 *defined by the specified vertex.
 *@param _g the graph
 *@param ccedges container(vector) where the edges will be stored; this can be 
 *vector<pair<VID,VID>> for unweighted or vector<pair<pair<VID,VID>,Weihght> for weighted Graphs
 *@param _v1 the vertex id that define the connected component
 *@note This returned edge list might contains duplicate edges. this
 *is when v1->v2 was in the list, v2->v1 might also be in the list.
 *
 *Here v1 is the vertex constains user data in first parameter,
 *v2 is the vertex constains user data in second parameter.
 *if there are more than one, then the first will be applied.
 *
 *@return the number of elements pushed in ccedges
 *@see BFS(VID) and GetIncidentEdgesVData(VID)
 */
template<class GRAPH, class CONTAINER>
int GetCCEdgesVData (GRAPH& _g, 
             CONTAINER& ccedges,
             VID  _v1id)  {
  CONTAINER newedges;
  
  vector<VID> ccverts;
  BFSVID(_g,_v1id, ccverts);
  sort(ccverts.begin(),ccverts.end());
  for (int i=0; i < ccverts.size(); i++) {
    _g.GetIncidentEdgesVData(ccverts[i],newedges);
    for (int k=0; k < newedges.size(); k++){
      ccedges.push_back ( newedges[k] );
    }
  }
  return ccedges.size();
}

//TESTED
/**Get user data associated with edges in connected conponent 
 *defined this specified vertex.
 *@param _g the graph
 *@param ccedges container(vector) where the edges will be stored; this can be 
 *vector<pair<VID,VID>> for unweighted or vector<pair<pair<VID,VID>,Weihght> for weighted Graphs
 *@param _v1 the vertex that define the connected component(as user data)
 *@note This returned edge list might contains duplicate edges. this
 *is when v1->v2 was in the list, v2->v1 might also be in the list.
 *
 *Here v1 is the vertex constains user data in first parameter,
 *v2 is the vertex constains user data in second parameter.
 *if there are more than one, then the first will be applied.
 *
 *@return the number of elements pushed in ccedges
 *@see BFS(VID) and GetIncidentEdgesVData(VID)
 */
template<class GRAPH, class CONTAINER>
int GetCCEdgesVData ( GRAPH& g, 
              CONTAINER& ccedges,
              typename GRAPH::VERTEX_TYPE _v1) {
    return GetCCEdgesVData ( g, ccedges, g.GetVID(_v1) );
}


// return 2D vector ccedges[i,j] = jth edge of ith CC, edge is VERTEX pair
//TESTED
/**Get a list which contains a list of edges in same Connected Components.
 *If there are n components in this graph then there will be n edge lists in
 *this returned list.
 *@param _g the graph
 *@param ccedges 2D vector ccedges[i,j] = jth edge of ith CC, edge is VERTEX pair.
 *@return the number of edges pushed in ccedges
 *@see GetCCStats
 */
template<class GRAPH, class CONTAINER>
int GetEdgesByCCVDataOnly(GRAPH& g, vector<CONTAINER> & ccedges) {
  //this has the semantic a little bit modified because it will 
  //accept and return different containers depending on the weighted/unweighted property
  //also is not tested yet
  vector< pair<int,VID> > cc;
  GetCCStats(g,cc);
  for (int i=0; i < cc.size(); i++) {
    CONTAINER thiscc;
    GetCCEdgesVData(g, thiscc, cc[i].second);
    
    /* //THE COMMENTED CODE WILL GIVE A COMPILER ERROR
    for (int k=0; k < thiscc.size(); k++){
      ccedges.push_back ( thiscc[k] );
    }*/
    
    ccedges.push_back ( thiscc );
    thiscc.clear();
  }
  return ccedges.size();
}


//==================================
// WeightedGraph class Predicates, Comparisons & Operations
//==================================
template <class T>
struct __CCVID_Compare : public binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.first > y.first; }
};

/**Get all Connected Components from this graph.
 *@return A list of pairs. The dirst element in each pair is the size
 *of component and the second element is the vertex defines this component.
 *@param _g the graph
 *@param _ccstats vector where the statistic will be stored
 *@return the number of components pushed in _ccstats
 *@note the list is ordered by the size of component, the first element in
 *the list has largest size.
 *@see CCVID_Compare
 */
template <class GRAPH>
int GetCCStats (GRAPH& _g,vector< pair<int,VID> >& _ccstats) {
  //!!!!!!!!!!!!!!!!! this crash for directed; see what is wrong
  //get vertices vid
  typedef typename GRAPH::VERTEX_TYPE GV;
  typedef typename GRAPH::WEIGHT_TYPE GW;

  typedef Graph<DG<GV,GW>,MG<GV,GW>,WG<GV,GW>,GV,GW> DGRAPH;
  
  //vector< pair<int,VID> > ccstats;
  _ccstats.clear();
  vector<VID> verts;
  _g.GetVerticesVID(verts);
  
  while ( verts.size() != 0 ) {
    VID v1id = verts.front();
    //the next three lines are from old graph
    //DGRAPH bfstree;
    //true_BFS(g, bfstree, v1id);
    //vector<VID> CC = bfstree.GetVerticesVID();
    vector<VID> CC;
    CC.clear();
    GetCC(_g,v1id,CC);

    int CCsize = CC.size();
    _ccstats.push_back( pair<int,VID>(CCsize,v1id) );
    sort( CC.begin(), CC.end() ); //sort by VID for set_difference
    set_difference
      ( verts.begin(),verts.end(),CC.begin(),CC.end(),verts.begin() );
    verts.erase(verts.end()-CC.size(), verts.end());
  }
    
  sort ( _ccstats.begin(), _ccstats.end(), __CCVID_Compare<pair<int,VID> >() );
  
  return _ccstats.size();
}
/**Get number of Connected Components in this graph.
 *@param _g the graph
 *@see GetCCStats
 */
template <class GRAPH>
int GetCCcount (GRAPH& _g) {
  vector< pair<int,VID> > ccstats;
  return GetCCStats(_g,ccstats);
}


//==================================
// WeightedGraph class Methods: Display, Input, Output 
//==================================
/**Output Connected Component defined by VID information to standard output.
 *This method output every VID in this Connected Component.
 *Following format will be output, CC[vids] ={ (v1,v2),... }
 *@param g the graph
 *@param _v1id this id defines a Connected Component(the one it belongs).
 *@see GetCC(VID)
 */
template <class GRAPH>
void DisplayCC ( GRAPH& g, VID _v1id) {
  typedef vector<VID>::iterator VI;
    
  vector<VID> ccverts;
  GetCC(g, _v1id, ccverts);
  cout << "\nCC[" << _v1id << "] = {";
  for (VI vi = ccverts.begin(); vi < ccverts.end(); vi++ ) {
    cout << *vi; 
    if (vi != ccverts.end() -1 ) cout << ", ";
  }
  cout << "}\n";
}


/**Output edges in connected components.
 *Following format will be output, CC[i] ={ (v1,v2),... }
 *here i is ith connected component in the graph.
 *v1 and v2 are user data associated with a edge.
 *@param _g the graph
 *@see GetEdgesByCCVDataOnly
 */
template <class GRAPH>
void DisplayEdgesByCCVDataOnly(GRAPH& g) {

  vector< vector< pair<typename GRAPH::VERTEX_TYPE, typename GRAPH::VERTEX_TYPE> > >  ccedges;
  GetEdgesByCCVDataOnly(g,ccedges);
  cout << endl << "Edges in each connected component (vertex data shown)";
  for (int cc=0; cc < ccedges.size(); cc++){
    cout << endl << "CC[" << cc << "]: ";
    for (int e=0; e < ccedges[cc].size(); e++){ 
      cout << " (" << ccedges[cc][e].first << "," << ccedges[cc][e].second << ")"; 
    }
  }
}

/**Output Connected Component information in graph _G.
 *This method will print out _maxCCprint number of Connected Component,
 *,and size and start vertex of each Connected Component.
 *@param _g the graph
 *@param _maxCCprint the number of connected components to print.
 */
template <class GRAPH>
void DisplayCCStats(GRAPH& g, int _maxCCprint=-1)  {
    
    ///Modified for VC
    typedef vector< pair<int,VID> > PAIR_VECTOR;
    typedef PAIR_VECTOR::const_iterator CCI; 
    
    int maxCCprint;
    
    vector< pair<int,VID> > ccstats;
    GetCCStats(g, ccstats);
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


////////////////////////////
////// DIJKSTRA ALGORITHM//
///////////////////////////


class dkinfo
{

public:
    bool VertexValid; //Keeps track of whether the second vertex is valid or not
                    //In previous Graph.h we used INVALID_VID to do the same purpose

    VID vid;    //This vertex ID
    // If the VertexValid flag is set to false then the next two variables does not have any meaning and should not be used
    
    VID predvid; //ID of the previous vertex in the graph
    float dist; //Distance between the two vertices.

    dkinfo()
    {
        VertexValid=false;
        vid=0;
        predvid=-1000;
        dist=-1000;
    }
    
    dkinfo(bool flag,VID _vid, VID _pvid,float _dist)
    {
        VertexValid=flag;
        vid=_vid;
        predvid=_pvid;
        dist=_dist;
    }
    
    void print() const
    {
        cout<<"Vertex : "<<vid
            <<"  Prev Vertex : "<<predvid
            <<"  Distance : "<<dist
            <<"  Valid ? "<<VertexValid<<endl;
    }
    
    bool valid() const
    {   return VertexValid;
    }

};


#ifdef __HP_aCC
       static bool dkinfo_compare (const dkinfo d1, const dkinfo d2)
#else 
       static bool dkinfo_compare (const dkinfo& d1, const dkinfo& d2)
#endif
{
    //d1.print();
    //d2.print();
    if ((!d1.valid())&&(!d2.valid()))
    {
        //cout<<"Both d1 and d2 are not valid"<<endl;
        //The returned value does not make much sense
        return ( d1.dist > d2.dist );
    }
    else if(!d1.valid())
    {
        //cout<<"D1 not valid"<<endl;
        //So d1.dist > d2.dist
        return true;
    }
    else if(!d2.valid())
    {
        //cout<<"D2 is not valid"<<endl;
        return false;
    }
    else
    {
        //cout<<"Both are valid"<<endl;     
        return d1.dist > d2.dist;
    }
    
}

/**Calculate a Dijkstra sssp tree.
 *returns a sssp tree where edge weights are distances from source
 *i.e., the edge weights are cumulative path lengths.
 *This tree starts from VID and to all vertices which are reachable from this vertex.
 *@param g the graph
 *@param ssptree the tree that will be built by the algorithm
 *@param start_VID the id of the starting vertex  
 */
template<class GRAPH,class R> 
void DijkstraSSSP(GRAPH g,R& sssptree,VID start_vid)
{


    typename GRAPH::WEIGHT_TYPE _w_;
        
    typename GRAPH::CVI cv1;
        typename GRAPH::VI v1;
    

    
    vector<dkinfo> pq; //The priority Queue
        float  maxdist = g.GetVertexCount() * _w_.MaxWeight();

        
        
        for ( cv1 = g.v.begin(); cv1 != g.v.end(); cv1++) 
        {
                        
            if (cv1->vid == start_vid) 
            {
                pq.push_back( dkinfo(true,cv1->vid,cv1->vid,0) );
            }
            else 
            {
              pq.push_back( dkinfo(false,cv1->vid,-1,-1000) );
            }
        }
        
    sort( pq.begin(), pq.end(),  (dkinfo_compare) );

    
        while ( pq.size() != 0  && (pq.back().valid()) ) 
        {
        
        bool relax = false;
            dkinfo u = pq.back();
            
            if ( sssptree.GetVertexCount() == 0 ) 
            {
                typename GRAPH::VERTEX_TYPE tmp = g.GetData(u.vid);
                sssptree.AddVertex( tmp );
            } 
            else 
            {
                    typename GRAPH::VERTEX_TYPE tmp = g.GetData(u.vid);
                    sssptree.AddVertex( tmp );
                    typename GRAPH::VERTEX_TYPE tmp1 = g.GetData(u.predvid);
                    sssptree.AddEdge( tmp1, tmp, u.dist);
            } 

            pq.pop_back();
            vector<VID> adj;
            g.GetDijkstraInfo(u.vid,adj);
            

            // check all u's successors 
            
            
            for (int i = 0; i < adj.size(); i++) 
            {
                for (int j=0; j < pq.size(); j++) 
                {
                    if (adj[i] == pq[j].vid) 
                    {
                        
                        float wt = g.GetEdgeWeight(u.vid,pq[j].vid).Weight();
                        
                       
                        if(!pq[j].valid())
                        {
                                relax = true;
                                pq[j].dist = u.dist + wt;
                                pq[j].predvid = u.vid;
                                pq[j].VertexValid=true;
                            
                        }
                        else if ( pq[j].dist > u.dist + wt ) 
                            { 
                                relax = true;
                                pq[j].dist = u.dist + wt;
                                pq[j].predvid = u.vid;
                            pq[j].VertexValid=true;
                            }
                        }
                    } // endfor
            } // endfor
              
            if (relax) sort( pq.begin(), pq.end(),dkinfo_compare);  

        }
    return ;    
}





template <class T>
struct less_cmp : public binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.second > y.second; }
};

#define UNSEEN 0
#define FRINGER 1
#define TNODE 2

/**
*Find the minimum weight path from source _v1id to the destination
*_v2id in Graph _g; The path is stored in vector _V
*@param _g the Graph
*@param _v1id start vertex id
*@param _v2id destination vertex id
*@param _V vector where the edges of the path will be stored
*@return The length of the path 
*@note _startV, _endV represent vertices id; This function works only for weighted graphs.
*/
template <class GRAPH>
int FindPathDijkstra (GRAPH& _g, VID _v1id, VID _v2id,
              vector< pair<typename GRAPH::VERTEX_TYPE,
              typename GRAPH::WEIGHT_TYPE> >& _V) {

  typedef typename GRAPH::WEIGHT_TYPE WT;
  typedef typename GRAPH::VERTEX_TYPE VT;

  map<VID,int> status;
  map<VID,float> weight;
  map<VID,float> dist;
  map<VID,VID> parent;
  typename GRAPH::CEI cei;
  typename GRAPH::VI vi;
  
  vector<pair<VID,float> > heap;

  //we will use the heap provided by STL; 
  //Actually we have a vector and the functions push_heap, pop_heap
  //VID i,w,u;
  VID u;
  WT t1;
  _V.clear();

  for(vi = _g.begin(); vi != _g.end(); vi++) {   
    status[vi->vid]=UNSEEN;
    dist[vi->vid]=0;
  }
  status[_v1id] = TNODE;
  //WtVertexType<typename GRAPH::VERTEX_TYPE,
  //  typename GRAPH::WEIGHT_TYPE> *cvi;
  typename GRAPH::VI cvi;
  if(!_g.IsVertex(_v1id,&cvi)) {
    cout<<"ERROR while trying to retrieve the CVI in FindPathDijkstra"<<endl;
    return -1;
  }
  //cvi = _g.my_find_VID_eqc(_v1id);
  for(cei = cvi->edgelist.begin(); cei != cvi->edgelist.end(); cei++) {
    //for every edge s->w
    VID w=cei->vertex2id;
    status[w] = FRINGER;    
    t1=cvi->GetEdgeWeight(cei->vertex2id);
    float wt=t1.Weight();        
    dist[w] = wt;
    parent[w]=_v1id;
    heap.push_back(pair<VID,float>(w,dist[w]));
  }
  make_heap(heap.begin(),heap.end(),less_cmp<pair<VID,float> >());
  while((status[_v2id] != TNODE)&&(heap.size()>0)){
    pop_heap(heap.begin(),heap.end(),less_cmp<pair<VID,float> >());
    u=heap.back().first;//take the minimum
    heap.pop_back();//remove last element from dist
    status[u]=TNODE;
    //cvi = _g.my_find_VID_eq(u);
    if(!_g.IsVertex(u,&cvi)) {
      cout<<"ERROR while trying to retrieve the CVI in FindPathDijkstra"<<endl;
      return -1;
    }
    for(cei = cvi->edgelist.begin(); cei != cvi->edgelist.end(); cei++) {
      //for(i=0;i<ls[u].size();i++){
      //for every edge s->w
      VID w = cei->vertex2id;
      if(status[w]==UNSEEN){
    status[w]=FRINGER;
    parent[w]=u;
    t1=cvi->GetEdgeWeight(cei->vertex2id);
    float wt=t1.Weight();        
    dist[w]=dist[u] + wt;
    heap.push_back(pair<VID,float>(w,dist[w]));
    push_heap(heap.begin(),heap.end(),less_cmp<pair<VID,float> >());
      }
      else if(status[w]==FRINGER){
    t1=cvi->GetEdgeWeight(cei->vertex2id);
    float wt=t1.Weight();        
    if(dist[w] > (dist[u]+wt)){
      parent[w]=u;
      dist[w]=dist[u] + wt;
      // dist[w] = dist[u]+ls[u][i]->cap;
      heap.push_back(pair<VID,float>(w,dist[w]));
      push_heap(heap.begin(),heap.end(),less_cmp<pair<VID,float> >());
    }
      }
    }
  }
  if(status[_v2id] != TNODE) return -1;//there is no path
  //now prepare the return datastructure
  VID temp = _v2id;
  //push the last element 
  VT tmp2 = _g.GetData(_v2id);
  WT tempwt(-1);
  _V.push_back(pair<VT,WT>(tmp2,tempwt));
  while(parent[temp] != _v1id){
    VT tmp1 = _g.GetData(parent[temp]);
    WT wt = _g.GetEdgeWeight(parent[temp],temp);
    _V.push_back(pair<VT,WT>(tmp1,wt));
    temp = parent[temp];
  }
  //push the first element
  VT tmp1 = _g.GetData(parent[temp]);
  WT wt = _g.GetEdgeWeight(parent[temp],temp);
  _V.push_back(pair<VT,WT>(tmp1,wt));
  reverse(_V.begin(), _V.end());
  status.clear();
  weight.clear();
  dist.clear();
  heap.clear();
  return _V.size();
}

///////////////////////////////////////////////////////////////////////
// FindPathDijkstra
// *NOTE* returns shortest path where edge weights are original edge weights
///////////////////////////////////////////////////////////////////////

/**

*Find the minimum weight path from source _startV to the destination
*_endV in Graph _g;
*@param _g the Graph
*@param _startV starting point represented as user data 
*@param _endV destination point represented as user data 
*@param _V vector where the edges of the path will be stored
*@return The length of the path 
*@note _startV, _endV represent user data
*/
template <class GRAPH>
int FindPathDijkstra (GRAPH& _g,
              typename GRAPH::VERTEX_TYPE& _startV, 
              typename GRAPH::VERTEX_TYPE& _endV,
              vector< pair<typename GRAPH::VERTEX_TYPE,typename GRAPH::WEIGHT_TYPE> >& _V) {
  typename GRAPH::CVI cv1,cv2;
  _V.clear();
  if ( _g.IsVertex(_startV,&cv1) && _g.IsVertex(_endV,&cv2) ){
    FindPathDijkstra(_g,cv1->vid,cv2->vid,_V);
    return _V.size();
  } else {
    cout << "\nIn FindPathDijkstra: start or goal vertex (";
    cout << _startV << ", " << _endV << ") not in graph";
    return 0;
  }
}




//the next algorithm computes strongly connected components;
//it is not fully tested yet;

template <class GRAPH,class CONTAINER>
class _visitor_SCC: public visitor_base<GRAPH>{
  CONTAINER* finishtimes;
  int finish;
 public:  
  _visitor_SCC(GRAPH g, CONTAINER* _v){
   finishtimes = _v;
   finishtimes->resize(g.size());
   finish = 0;
  }
  
  int finish_vertex(typename GRAPH::VI vi){
    (*finishtimes)[vi->vid].first  = vi->vid;
    (*finishtimes)[vi->vid].second = finish++;
    return OK;
  }

};


template <class T>
struct __SCC_Compare : public binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.second > y.second; }
};






template <class GRAPH>
int dfs_pred(GRAPH& _g,map<VID,int>& _color,vector<vector<VID> >& _scc,VID _start, int scc_no ){

  typename GRAPH::VI vi;
  _color[_start] = GRAY;
  _scc[scc_no].push_back(_start);
  
  if(!_g.IsVertex(_start,&vi)){
    cout<<"ERROR:: inside dfs_pred"<<endl;
    return ERROR;
  }
     
  for(typename GRAPH::EI ei = vi->predecessors.begin();ei!= vi->predecessors.end();ei++){
    if(_color[ei->vertex2id] == WHITE){
      dfs_pred(_g,_color,_scc,ei->vertex2id,scc_no);
    }
  }
  
}


template <class GRAPH>
int scc(GRAPH& _g, vector<vector<VID> >& _scc){
  vector<pair<VID,int> > ft;
  int i;
  int scc_no=0;

  
  _visitor_SCC<GRAPH, vector<pair<VID,int> > > _vis(_g,&ft);
  DFS_functor(_g ,_vis);
  //for(i=0;i<ft.size();i++)
  //  cout<<"["<<ft[i].first<<" "<<ft[i].second<<"]"<<endl;
  _g.SetPredecessors();
  //here a dFS on reverse using ft
  cout<<endl;
  map<VID, int> color;
  for(i=0;i<_g.size();i++) color[i] = WHITE;
  sort(ft.begin(),ft.end(),__SCC_Compare<pair<VID,int> >());
  //for(i=0;i<ft.size();i++)
  //  cout<<"["<<ft[i].first<<" "<<ft[i].second<<"]"<<endl;

  vector<VID> _t;
  for(i=0;i<ft.size();i++){
    if(color[ft[i].first] == WHITE){
      _scc.push_back(_t);
      dfs_pred(_g,color,_scc,ft[i].first,scc_no);
      scc_no++;
    } 
  }
  return OK;
}
};

#endif


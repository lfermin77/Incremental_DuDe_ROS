
#include "bpc.h"
#include "dp_approx.h"
#include "poly_approx.h"
#include "chull.h"
#include "intersection.h"
#include "dude_util.h"
#include <set>
#include <map>
#include <vector>

c_BPC::c_BPC()
{
    m_best_kid=NULL; //the kid that has the largest concavity
    m_parent=NULL;
//    m_concavity=NULL;
    m_source[0]=m_source[1]=NULL;
    m_next=NULL;
    m_pre=NULL;
}


//
// build: determine the order of bridge vertices and compute the length of pocket
//
bool c_BPC::build(ply_vertex * s, ply_vertex * e)
{
    //determine order
    char code=checkBridge(s,e);

    if(code=='0') return false;

    if(code=='r'){
        swap(s,e);
        code=checkBridge(s,e); //check again...
        if(code!='b') return false;
    }

    m_source[0]=s;
    m_source[1]=e;

    if(s->getNext()==e) return false; //no gap...

    //compute length
    m_pocketsize=0;
    ply_vertex * ptr=s;
    do{
        m_pocketsize++;
        ptr=ptr->getNext();
    }
    while(ptr!=e->getNext());

    return true;
}

////this 
//bool c_BPC::buildCompelete_for_hole(ply_vertex* s,ply_vertex* e)
//{
//
//}
//
// build: determine the order of bridge vertices and compute the length of pocket
//
bool c_BPC::build_for_hole(ply_vertex * s, ply_vertex * e, bool ordered)
{
    //determine order, the bridge should enclose a small area if possible
    if(ordered==false)
    {
        float area1=bridgeArea(s,e);
        float area2=bridgeArea(e,s);

        if(area1>0 || area2>0) return false; //should be both negative...

        if(area1<area2){ //area1 is more negative...
            swap(s,e);
        }
    }

    m_source[0]=s;
    m_source[1]=e;

    if(s->getNext()==e) return false; //no gap...

    //compute length
    m_pocketsize=0;
    ply_vertex * ptr=s;
    do{
        m_pocketsize++;
        ptr=ptr->getNext();
    }
    while(ptr!=e->getNext());

    return true;
}


////collect kid BPC from pocket vertices
////If there no BPC for a vertex, register this BPC
//void c_BPC::collectKids()
//{
//    //do it for tail and head first
//    m_source[0]->getExtra().bpc=this;
//    m_source[1]->getExtra().bpc=this;
//
//    //do it for the interval
//    ply_vertex * ptr=m_source[0]->getNext();
//    set<c_BPC*> kids;
//    while(ptr!=m_source[1])
//    {
//        ply_vertex_extra& extra=ptr->getExtra();
//        if(extra.bpc!=NULL)
//        {
//            c_BPC * root=extra.bpc;
//            if(root!=this)
//                kids.insert(root); //add this root to my kid
//        }
//        extra.bpc=this;
//        ptr=ptr->getNext();
//    }
//
//    for(set<c_BPC*>::iterator i=kids.begin();i!=kids.end();i++)
//        addKid(*i);
//}
void c_BPC::initPointList()
{
	//put its own start and end point
	//kidStartPnts.push_back(this->getSource1());
	//kidEndPnts.push_back(this->getSource2());
	for(list<c_BPC *>::iterator it=m_kids.begin();it!=m_kids.end();it++)
	{
		c_BPC * kid=*it;
		kidStartPnts.push_back(kid->getSource1());
		kidEndPnts.push_back(kid->getSource2());
	}
	
}
int c_BPC::checkStartPoint(ply_vertex* curVert)
{
	int resIdx = 0;
	vector<ply_vertex*>::iterator vit = kidStartPnts.begin();
	for (;vit!=kidStartPnts.end();++vit,++resIdx)
	{
		if(curVert == (*vit))
			return resIdx;
	}
	return -1;
}
int c_BPC::checkEndPoint(ply_vertex* curVert)
{
	int resIdx = 0;
	vector<ply_vertex*>::iterator vit = kidEndPnts.begin();
	for (;vit!=kidEndPnts.end();++vit,++resIdx)
	{
		if(curVert == (*vit))
			return resIdx;
	}
	return -1;
}

void c_BPC::getConvexApproximate(list<ply_vertex*>& approxList)
{
	//put all the points into the list
	initPointList();//initialize the start point list and end point list
	list<ply_vertex*> vList;
	ply_vertex * cursorPnt = this->getSource1();
	vList.push_back(cursorPnt);
	do{
		int sIdx = checkStartPoint(cursorPnt);
		//int eIdx = checkEndPoint(cursorPnt);
		if((sIdx!=-1))/*&&(cursorPnt!=this->getSource1()))*/
		{
			cursorPnt = kidEndPnts[sIdx];
		}
		else
			cursorPnt = cursorPnt->getNext();
		
		vList.push_back(cursorPnt);
	}while (cursorPnt!=this->getSource2());

	//split the list into several parts 
	list<ply_vertex*>::iterator curIt = vList.begin();

	while(curIt!=vList.end())
	{
		//use the start points and end points to split the line into several parts, and approximate each part individually
		list<ply_vertex*> tempList;
		int tmpEndIdx = checkEndPoint(*curIt);
		int tmpStartIdx = checkStartPoint(*curIt);
		if(tmpStartIdx!=-1)//if the current point is a start point, skip this kid pocket
			++curIt;
		if(curIt==vList.end())
			break;
		while ((curIt!=vList.end())&&(checkStartPoint(*curIt)==-1))
		{
			tempList.push_back(*curIt);
			++curIt;
		}
		//approximate the current plyline
		if (tempList.size()>2)
		{
			list<ply_vertex*> approx;
			map<ply_vertex*,ply_vertex*> vmap, vmap2;
			//create a ply_line for this vertex list
			c_plyline tmp = bpc2polyline(tempList);
			list<ply_vertex*> hull;
			hull2d(tmp.getHead(),tmp.getTail(),hull);

			c_plyline hullply= bpc2polyline(hull);
			//compute the approximate c_ply for the hull.
			//the approximate is now only deal with c_plyline,
			//needed to be modified.
			
			//approximate(hullply,m_tau-m_tau/4,approx);
			approximateMedthod(hullply,m_tau-m_tau/4,approx);

			//map from hullply to tmp (store in "hull")
			/////
			//Here mapping the convex hull points to the points on the converted polyline
			//
			{
				ply_vertex *j=hullply.getHead();
				for(list<ply_vertex*>::iterator i=hull.begin();i!=hull.end();i++){
					vmap2[j]=*i;
					j=j->getNext();
				}
			}
			//map from tmp (store in "tmp") to the original polygon
			//
			{
				ply_vertex *j=tmp.getHead();
				for(list<ply_vertex*>::iterator i=tempList.begin();i!=tempList.end();i++){
					vmap[j]=*i;
					j=j->getNext();
				}
			}
			//mark the approximation
			for(list<ply_vertex*>::iterator ait = approx.begin();
				ait!=approx.end();++ait)
			{
				ply_vertex* v = vmap[vmap2[*ait]];
				assert(v);

				approxList.push_back(v);
				//v->getExtra().flag = marking_flag;
			}
		}
		else
		{
			for (list<ply_vertex*>::iterator pit = tempList.begin();pit!=tempList.end();++pit)
			{
				approxList.push_back(*pit);
			}
			
		}
	}
	
}


void c_BPC::determineConcavity_real(float tau,bool reducePMs)
{
	m_tau = tau;

	typedef list<ply_vertex*>::iterator IT;
	typedef list<ply_vertex*>::const_iterator CIT;

	list<ply_vertex*> PMs;

	//visualize the parent pocket
	c_ply tmpply = seg2Ply(this->getSource1(),this->getSource2());
//	parentPck.push_back(tmpply);

	vector<c_ply> tmpChildren;//visualize the children pocket
	for(list<c_BPC *>::iterator it=m_kids.begin();it!=m_kids.end();it++)
	{
		c_BPC * kid=*it;
		kid->determineConcavity(tau);

		///****add by Guilin************/
		//PMs.push_back(kid->getSource1());
		//PMs.push_back(kid->getSource2());
		///****************************/

		const list<ply_vertex *>& kid_concavities=kid->getConcavities();
		for(CIT j=kid_concavities.begin();j!=kid_concavities.end();j++){
			//v->getExtra().concavity=dist_in_hierarchy(v->getExtra().concavity_bpc,v->getPos());
			PMs.push_back(*j);
		}

		//////////////////////////////////////////////////////////////////////////
		//add the pocket children to the visualization list
		c_ply tmpply = seg2Ply((*it)->getSource1(),(*it)->getSource2());
		tmpChildren.push_back(tmpply);
	}
//	childrenPck.push_back(tmpChildren);//add to the visualization list

		//
		// convert the bridge to polygon that contains no child pockets
		// convert the polygon to its convex hull
		// then approximate convex hull
		//
		list<ply_vertex*> vlist, approx;
		map<ply_vertex*,ply_vertex*> vmap, vmap2;
		
		getVerticesExcludingKidPockets_f(vlist);
		c_plyline tmp=bpc2polyline(vlist);
	
		//////////////////////////////////////////////////////////////////////////
	    list<ply_vertex *> hull;
	    hull2d(tmp.getHead(),tmp.getTail(),hull);

	    c_plyline hullply=bpc2polyline(hull);
	
		//convex hull of rest polyline
//		allExPocPlys.push_back(tmp.toply());
		c_diagonal tBridge(this->getSource1(),this->getSource2());
//		allExPocBridges.push_back(tBridge);
	
	
	    //g_hullply=hullply;
	
		//approximate(hullply,tau-tau/4,approx);
	    approximateMedthod(hullply,tau-tau/4,approx);

//		allExPocApproxs.push_back(list2Ply(approx));
	
	
		//map from hullply to tmp (store in "hull")
		/////
		//Here mapping the convex hull points to the points on the converted polyline
		{
	        ply_vertex *j=hullply.getHead();
	        for(IT i=hull.begin();i!=hull.end();i++){
	            vmap2[j]=*i;
	            j=j->getNext();
	        }
		}
		
		//map from tmp (store in "tmp") to the original polygon
	    {
	        ply_vertex *j=tmp.getHead();
	        for(IT i=vlist.begin();i!=vlist.end();i++){
	            vmap[j]=*i;
	            j=j->getNext();
	        }
	    }

	// process the approximation

	//get the approximate list

	//list<ply_vertex*> approxList;
	//getConvexApproximate(approxList);

	for(IT i=approx.begin();i!=approx.end();i++){
	//for(list<ply_vertex*>::iterator i = approxList.begin();i!=approxList.end();++i)
	//{
		ply_vertex * v=vmap[vmap2[*i]];
		//ply_vertex * v = *i;
		assert(v);

		//make sure that the PM is not the end point of bridges
		if(v==m_source[0]||v==m_source[1]) continue; //cannot be end point of the bridge
		bool is_bridge=false;
		for(list<c_BPC *>::iterator k=m_kids.begin();k!=m_kids.end();k++){
			c_BPC * kid=*k;
			if(kid->getConcavities().empty()) continue;
			if(v==kid->m_source[0]||v==kid->m_source[1]){
				is_bridge=true;
				break;
			}
		}

		//commented by Guilin
		if(is_bridge) continue; //cannot be kids' bridge

		//compute concavity
		float concavity=dist2Bridge(v->getPos());

		//		if(v->getVID()==139)
		//		    cout<<"[139]: "<<concavity<<" bpc="<<this<<endl;

		if(concavity<tau) continue;
		//
		//v->getExtra().concavity=concavity;
		v->getExtra().concavity_bpc=this;

		PMs.push_back(v);
	}

	//	m_concavities=PMs;

	//	return;

	//post process the PMs since they can still be close to the end points of
	//the bridges
	list<ply_vertex *> features;
	getSortedFeatures_f(PMs,features);
	for(IT i=features.begin();i!=features.end();i++){

		//add by Guilin
		//m_concavities.push_back(*i);

		c_BPC * c_bpc=(*i)->getExtra().concavity_bpc;
		if( c_bpc==NULL ) continue; //not PM (bridge end points)
		if( c_bpc!=this ){
			m_concavities.push_back(*i); //not the PM of this bpc (but kids' PM)
		}
		else{
			IT pre=i; pre--;
			IT next=i; next++;
			assert(i!=features.begin());
			assert(next!=features.end());

			//m_concavities.push_back(*i);

			//check if the distance is OK...
			float d=dist2Seg((*pre)->getPos(),(*next)->getPos(),(*i)->getPos());
			if(d>tau/2) m_concavities.push_back(*i);
		}
	}
}
void c_BPC::determineConcavity(float tau,bool reducePMs)
{
	determineConcavity_real(tau,reducePMs);
}
c_ply c_BPC::seg2Ply(ply_vertex* vstart,ply_vertex* vend)
{
	c_ply ply(c_ply::UNKNOWN);
	ply_vertex * tmpv = vstart;
	while (tmpv!=vend->getNext())
	{
		const Point2d& pos = tmpv->getPos();
		ply.addVertex(pos[0],pos[1]);
		tmpv = tmpv->getNext();
	}

	ply.endPoly();	
	return ply;
}
c_ply c_BPC::list2Ply(list<ply_vertex*> & vertlist)
{
	c_ply ply(c_ply::UNKNOWN);
	list<ply_vertex*>::iterator vit = vertlist.begin();
	for (;vit!=vertlist.end();++vit)
	{
		const Point2d& pos = (*vit)->getPos();
		ply.addVertex(pos[0],pos[1]);
	}
	ply.endPoly();	
	return ply;
}
void c_BPC::determine_PM(float tau)
{
    typedef list<ply_vertex*>::iterator IT;

    list<ply_vertex*> tmp;
    tmp.swap(m_concavities); //now m_concavities becomes empty

    for(IT i=tmp.begin();i!=tmp.end();i++){
        ply_vertex * v=*i;

        //compute concavity
        ply_vertex_extra & extra=v->getExtra();
        float concavity= dist2Bridge(v->getPos()); //dist_in_hierarchy(extra.concavity_bpc,v->getPos());

       // float concavity=this->dist2Bridge(v->getPos());
        if(concavity<tau) continue;

//        cout<<"["<<v->getVID()<<"], concavity="<<concavity<<" bpc="<<this<<endl;

        extra.concavity=concavity;
        m_concavities.push_back(v);
    }
}

//
// TODO: tip is a vertex between two BPSc and farthest
// away from the end points of the BPSc
//
/*
void c_BPC::determineTip()
{
    if(m_next==NULL) return;
    ply_vertex * s=m_source[1];  //end of this pocket
    ply_vertex * e=m_next->m_source[0]; //begin of the next pocket

    float max_d=0;
    ply_vertex * max_v=s;

    ply_vertex * ptr=s->getNext();
    while(ptr!=e->getNext())
    {
        float d=dist2Line(s->getPos(),e->getPos(),ptr->getPos());
        if(d>max_d){
            max_d=d;
            max_v=ptr;
        }
        ptr=ptr->getNext();
    }

    m_tip=max_v;
}
*/

//check if two PBCs overlap
bool c_BPC::isOverlapping(c_BPC * other)
{
    ply_vertex * s=getSource1()->getNext();
    ply_vertex * t=getSource2();
    ply_vertex * u=other->getSource1()->getNext();
    ply_vertex * v=other->getSource2();

    assert(other->getConcavities().empty()==false);
    ply_vertex * o=other->getConcavities().front();

    for(ply_vertex * ptr=s->getNext();ptr!=t;ptr=ptr->getNext()){
        if(ptr==o) return true;
    }

    for(ply_vertex * ptr=u->getNext();ptr!=v;ptr=ptr->getNext()){
        if(ptr==o) return true;
    }

    return false;
}


float c_BPC::dist_in_hierarchy(c_BPC * bpc, const Point2d& p)
{
    if(bpc->m_parent==NULL){
        return bpc->dist2Bridge(p);
    }
    else{
        float d=bpc->dist2Bridge(p);
        Point2d new_p=proj2Seg(bpc->m_source[0]->getPos(),bpc->m_source[1]->getPos(),p);
        return dist_in_hierarchy(bpc->m_parent,new_p)+d;
    }
}

//compute Euclidean distance
float c_BPC::dist2Bridge(const Point2d& p)
{
    return dist2Seg(m_source[0]->getPos(),m_source[1]->getPos(),p);
}

Point2d c_BPC::proj2Seg(const Point2d& s, const Point2d& t, const Point2d& p)
{
    Vector2d v = s-t;
    Vector2d u = p-t;
    float v_norm=v.norm(); //is also v_norm
    if(v_norm==0) return s;

    float d=(u*v)/v_norm;
    if(d<0){
        return t;
    }
    if(d>v_norm){
        return s;
    }

    Point2d tmp=t+v*(d/v_norm);
    return tmp;
}

//
//moved to dude_util.h"
//
//float c_BPC::dist2Seg(const Point2d& s, const Point2d& t, const Point2d& p)
//{
//    Vector2d v = s-t;
//    Vector2d u = p-t;
//    Vector2d n (v[1],-v[0]);
//    float n_norm=n.norm(); //is also v_norm
//    if(n_norm==0) return u.norm();
//
//    float d=(u*v/n_norm);
//    if(d<0){
//        return u.norm();
//    }
//    if(d>n_norm){
//        return (p-s).norm();
//    }
//
//    return fabs(n*u/n_norm);
//}


float c_BPC::bridgeArea(ply_vertex * p1, ply_vertex * p2)
{
    c_ply ply(c_ply::UNKNOWN);
    ply_vertex *ptr=p1;
    ply.beginPoly();
    do{
        const Point2d& pos=ptr->getPos();
        ply.addVertex(pos[0],pos[1],false);
        ptr=ptr->getNext();
    }while(ptr!=p2->getNext());

    ply.endPoly();

    float area=ply.getArea();
    ply.destroy();

    return area;
}

//
//
//
char c_BPC::checkBridge(ply_vertex * p1, ply_vertex * p2)
{
    if(p1->getNext()==p2 && (p1->isReflex()||p2->isReflex()) ) return 'b';
    if(p2->getNext()==p1 && (p1->isReflex()||p2->isReflex()) ) return 'r';

    float area=bridgeArea(p1,p2);

    char code='b';
    if(area<=1e-10 && area>=-1e-10) code = '0';
    else{
        if(area>0) code='r';
    }

    return code;
}


//
// get vertices that does not include the kids
//
void c_BPC::getVerticesExcludingKidPockets_f(list<ply_vertex*>& vlist)
{
	vlist.clear();
    uint flag=ply_vertex_extra::getFlagID();;//ply_vertex_extra::getFlagID();

//    if(this->getPocketSize()==3)
//        int a=0;

	for(list<c_BPC *>::iterator i=m_kids.begin();i!=m_kids.end();i++){
		c_BPC * kid=*i;

		if(kid->getConcavities().empty()) continue;

		for(ply_vertex * k=kid->m_source[0]->getNext();k!=kid->m_source[1];k=k->getNext()){
			//k->getExtra().flag=flag;
			k->getExtra().flag = flag;
		}
		
	}//end for

	m_source[0]->getExtra().flag=0;
	m_source[1]->getExtra().flag=0;

	int k=0,j=0;
	for(ply_vertex * i=m_source[0];i!=m_source[1];i=i->getNext(),k++)
	{
	    if(i->getExtra().flag==flag) 
			continue;
	    vlist.push_back(i);
		j++;
	}//end for
	vlist.push_back(m_source[1]);

	//cout<<"excluding vertices number: "<<k- j<<endl<<endl;
	//cout<<"all left vertices number: "<<j<<endl;

//	if(vlist.size()==2){
//	    int a=0;
//	    vlist.clear();
//	    getVerticesExcludingKidPockets(vlist);
//	}
}

void c_BPC::getSortedFeatures_f(list<ply_vertex*>& PMs, list<ply_vertex*>& features)
{
    uint flag=ply_vertex_extra::getFlagID();
    for(list<c_BPC *>::iterator k=m_kids.begin();k!=m_kids.end();k++)
    {
        c_BPC * kid=*k;
        kid->m_source[0]->getExtra().flag=kid->m_source[1]->getExtra().flag=flag;
    }

    typedef list<ply_vertex*>::iterator IT;
    for(IT i=PMs.begin();i!=PMs.end();i++)
    {
        ply_vertex * v=*i;
        v->getExtra().flag=flag;
    }

    m_source[0]->getExtra().flag=m_source[1]->getExtra().flag=flag;
    for(ply_vertex * ptr=m_source[0];ptr!=m_source[1];ptr=ptr->getNext())
    {
        if(ptr->getExtra().flag==flag)
            features.push_back(ptr);
    }
    features.push_back(m_source[1]);
}

c_plyline c_BPC::bpc2polyline( const list<ply_vertex*>& vlist )
{	
	typedef list<ply_vertex*>::const_iterator IT;
	c_plyline ply;
	ply.beginPoly();
	for(IT i=vlist.begin();i!=vlist.end();i++){
		const ply_vertex* v=*i;
		const Point2d& pos=v->getPos();
		ply.addVertex(pos[0],pos[1]);
	}
	
	ply.endPoly();
	
	return ply;
}


void c_BPC::reorganize_kids_f()
{
	uint count=1;
	for(ply_vertex * i=m_source[0];i!=m_source[1];i=i->getNext())
		i->getExtra().flag=count++; //dangerous way of using flag since the value is not from ply_vertex_extra::getFlagID
	                                //need to reset after use
	m_source[1]->getExtra().flag=count++;

	//
	list< pair<int,c_BPC *> > sorted_kids;
	for(list<c_BPC *>::iterator i=m_kids.begin();i!=m_kids.end();i++){
	    c_BPC * kid=*i;
	    int d=kid->getPocketSize();
	    sorted_kids.push_back(make_pair(d,kid));
	    m_bridge_ends.push_back(kid->m_source[0]);
	    m_bridge_ends.push_back(kid->m_source[1]);
	}
	sorted_kids.sort();//sorted_kids.begin(),sorted_kids.end());
	m_kids.clear();

	typedef list< pair<int,c_BPC *> >::iterator IT;
	for(IT i=sorted_kids.begin();i!=sorted_kids.end();i++)
	{
		c_BPC * kid=i->second;
		insert2tree(kid);
	}//end for	


	//reset all flags to 0
    for(ply_vertex * i=m_source[0];i!=m_source[1]->getNext();i=i->getNext())
        i->getExtra().flag=0;
    m_source[1]->getExtra().flag=0;
}

void c_BPC::insert2tree(c_BPC * other)
{
    uint other_flag0=other->m_source[0]->getExtra().flag;
    uint other_flag1=other->m_source[1]->getExtra().flag;
    typedef list<c_BPC *>::iterator IT;
    bool update_myself=false;
    bool add_to_myself=true;
    for(IT i=m_kids.begin();i!=m_kids.end();i++)
    {
        c_BPC * kid=*i;
        uint kid_flag0=kid->m_source[0]->getExtra().flag;
        uint kid_flag1=kid->m_source[1]->getExtra().flag;
        if(kid_flag1<=other_flag0 || kid_flag0>=other_flag1) continue; //no overlap
        if(kid_flag0<=other_flag0 && kid_flag1>=other_flag1){ //kid contains other
            kid->insert2tree(other);
            add_to_myself=false; //already added to kid
            break;
			//once modified to be "return;" by Guilin
        }
        else if(other_flag0<=kid_flag0 && other_flag1>=kid_flag1){
            other->insert2tree(kid);
            update_myself=true; //kid is added to other, need to update myself
        }
//        else{
//
//        	cerr<<other->m_source[0]->getPos()[0]<<", "<<other->m_source[0]->getPos()[1]<<endl;
//        	cerr<<other->m_source[1]->getPos()[0]<<", "<<other->m_source[1]->getPos()[1]<<endl;
//
//        	cerr<<kid->m_source[0]->getPos()[0]<<", "<<kid->m_source[0]->getPos()[1]<<endl;
//        	cerr<<kid->m_source[1]->getPos()[0]<<", "<<kid->m_source[1]->getPos()[1]<<endl;
//
//        	bool b1 = (other->m_source[0]->getPos() == kid->m_source[0]->getPos());
//
//        	assert(false);
//
//        }
    }

    if(update_myself){
        list<c_BPC *> tmp;
        tmp.swap(m_kids);
        //only keep kids with me as parent
        for(IT i=tmp.begin();i!=tmp.end();i++){
            c_BPC * kid=*i;
            if(kid->m_parent==this) m_kids.push_back(kid);
        }
    }

    if(add_to_myself){
        other->m_parent=this;
        m_kids.push_back(other);
    }
}

//this can be done in constant time check the vertex id...
bool c_BPC::isKid(c_BPC * other)
{
    bool s1_found=false;
    bool s2_found=false;

    for(ply_vertex * ptr=m_source[0];ptr!=m_source[1];ptr=ptr->getNext()){
        if(ptr==other->m_source[0]) s1_found=true;
        if(ptr==other->m_source[1]) s2_found=true;
    }

    if(m_source[1]==other->m_source[0]) s1_found=true;
    if(m_source[1]==other->m_source[1]) s2_found=true;

    if(s1_found && s2_found) return true;
    return false;
}


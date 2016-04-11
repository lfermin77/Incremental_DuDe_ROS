#include "poly_approx.h"
#include <vector>
#include <cassert>
#include <algorithm>
using namespace std;

list<Point2d> g_points;

struct c_chain
{
	c_chain(){ pm=s=e=NULL; dist=0; pre=next=NULL; }

	float distance(const Point2d& p){ assert(pm); return fabs((pm->getPos()-p)*normal); }

	float estimate_dist(ply_vertex * v1, ply_vertex * v2)
	{
	    assert(v1);
	    assert(v2);
        float d1=distance(v1->getPos());
        float d2=distance(v2->getPos());
        return max(d1,d2);
	}

	void recompute_dist(){
	    dist=estimate_dist(s,e);
	}

	bool overlap_next(){
	    if(next!=NULL) if(next->s==e) return true;
	    return false;
	}

	bool overlap_pre(){
        if(pre!=NULL) if(pre->e==s) return true;
        return false;
	}

	ply_vertex * pm;     //pocket minimum
	ply_vertex * s, * e; //start&end
	float dist;         //distance from (se) to pm
	c_chain * pre, * next;
	Vector2d normal;
};


bool chain_comp(const c_chain* c1, const c_chain* c2)
{
	return c1->dist>c2->dist;
}


//distance from p to (st)
inline float dist2Seg(const Point2d& s, const Point2d& t, const Point2d& p)
{
    Vector2d v = s-t;
    Vector2d u = p-t;
    Vector2d n (v[1],-v[0]);
    float n_norm=n.norm();

    if(n_norm==0) return 0;

    return fabs(n*u/n_norm);
}

//distance from p to line (x,n)
inline float dist2Line(const Point2d& x, const Vector2d& n, const Point2d& p)
{
    return fabs((x-p)*n);
}

//
// find the vertex between s and e furthest from s and e
//
ply_vertex * furthest(ply_vertex * s, ply_vertex * e)
{
	double max_dist=0;
	ply_vertex * max_v=e;
	ply_vertex * ptr=s;
	do{
		float d=dist2Seg(s->getPos(), e->getPos(), ptr->getPos());
		if(d>max_dist){
			max_dist=d;
			max_v=ptr;
		}
		ptr=ptr->getNext();
	}
	while(ptr!=e->getNext());
	
	return max_v;
}

inline bool expand(c_chain * chain)
{
	assert(chain);
	ply_vertex * t1=chain->s->getPre();
	ply_vertex * t2=chain->e->getNext();
	if(t1==NULL && t2==NULL) return false;
	
	//distance of chain when expand to left or right
	float d1=FLT_MAX, d2=FLT_MAX; 
	
	bool update1=false;
	bool update2=false;
	
	if(t1!=NULL){ //check if the chain can be expanded to t1
		d1=chain->distance(t1->getPos());
		update1=true;

		if(chain->overlap_pre()){ //overlapped, should cp give up cp->e?
		    c_chain * cp=chain->pre;
			float old_score=chain->distance(cp->e->getPos())+cp->distance(cp->e->getPos());
			float new_score=chain->distance(t1->getPos())+cp->distance(t1->getPos());
            if(new_score<old_score){
                if( cp->e==cp->pm ){
                    update1=false;
                }
            }
            else{
                update1=false;
            }
		}
	}
	
	if(t2!=NULL){
		d2=chain->distance(t2->getPos());
		update2=true;
		if(chain->overlap_next()){ //overlapped, should cn give up cn->s?
	        c_chain * cn=chain->next;
            float old_score=chain->distance(cn->s->getPos())+cn->distance(cn->s->getPos());
            float new_score=chain->distance(t2->getPos())+cn->distance(t2->getPos());
            if(new_score<old_score){
                if( cn->s==cn->pm ){
                    update2=false;
                }
            }
            else{
                update2=false;
            }
		}
	}
	
	if(update1 && update2 ){
		if(d1<d2) update2=false;
		else update1=false;
	}
	
	if(update1)
	{
		c_chain * cp=chain->pre;
		if(cp!=NULL){
			if(cp->e==chain->s){ //overlapped
				cp->e=cp->e->getPre();
				cp->recompute_dist();
			}
		}
		chain->dist=d1;
		chain->s=t1;
		return true;
	}
	else if(update2){
		c_chain * cn=chain->next;
		if(chain->next!=NULL){
			if(cn->s==chain->e){ //overlapped
				cn->s=cn->s->getNext();
				cn->recompute_dist();
			}
		}
		chain->dist=d2;
		chain->e=t2;
		return true;
	}
	
	return false;
}

//return max approximation error
//inline float approximate(const list<ply_vertex*>& seeds, list<ply_vertex*>& approx)
inline float approximate(const vector<c_chain *>& chains, list<ply_vertex*>& approx)
{
	//create a list of chains
	vector<c_chain *> open=chains;
	typedef list<ply_vertex*>::const_iterator IT;
	
	make_heap(open.begin(),open.end(),chain_comp);
	
	while(open.empty()==false)
	{
		pop_heap(open.begin(), open.end(), chain_comp);
		c_chain * best=open.back();
		open.pop_back();
		
//		cout<<"best chain="<<best<<" old dist="<<best->dist<<" ";

		//expand best
		bool expanded=expand(best);
		if(expanded){
			open.push_back(best);
//			cout<<"new dist="<<best->dist;
			make_heap(open.begin(), open.end(), chain_comp); //make heap b/c other chain may have changed...
		}

//		cout<<endl;
	}//end while
	
	//collect approximate
	approx.clear();
	float max_d=0;
	approx.push_back(chains.front()->s);
	for(vector<c_chain *>::const_iterator i=chains.begin();i!=chains.end();i++)
	{
//	    cout<<"final chains="<<(*i)->s<<", "<<(*i)->e<<endl;
		approx.push_back((*i)->e);
		if((*i)->dist>max_d) max_d=(*i)->dist;
	}
	
	return max_d;
}

//
// given a convex polygon (boundary), 
// return the approximation that has min(sum(d_i)) for all d_i<tau
// where d_i is the distance from the approximation to the input convex polygon
//
void approximate(const c_plyline& plyline, double tau, list<ply_vertex*>& approx)
{
	typedef list<ply_vertex*>::iterator IT;
	approx.push_front(plyline.getHead());
	approx.push_back(plyline.getTail());
		
	do{		

	    vector<c_chain *> chains;
	    list<ply_vertex*> new_approx=approx;

	    //add a new vertex to new_approx
	    float max_d=-FLT_MAX;
	    IT    insert_pt=new_approx.end();
	    ply_vertex* max_v=NULL;

	    for(IT i=new_approx.begin();i!=new_approx.end();i++){
            IT j=i; j++;
            if(j==new_approx.end()) break;
            ply_vertex * seed=furthest(*i,*j);
            if(seed == *j || seed == *i) continue;
            float d=dist2Seg( (*i)->getPos(),(*j)->getPos(),seed->getPos());
            if(d>max_d){
                max_d=d;
                max_v=seed;
                insert_pt=j;
            }
	    }

	    //g_points.push_back(max_v->getPos());

	    if(max_v!=NULL){

	        new_approx.insert(insert_pt,max_v);

			//move here by Guilin
			g_points.push_back(max_v->getPos());

	    }//max_v is not null

		//create chains
		for(IT i=new_approx.begin();i!=new_approx.end();i++){
			IT j=i; j++;
			if(j==new_approx.end()) break;
			ply_vertex * seed=furthest(*i,*j);

			//create chain from seed
            c_chain * chain=new c_chain();
            assert(chain);
            chain->pm=chain->s=chain->e=seed;
            Vector2d vec=( (*j)->getPos()-(*i)->getPos() ).normalize();
            chain->normal.set(vec[1],-vec[0]);
            if(chains.empty()==false){
                chain->pre=chains.back();
                chains.back()->next=chain;
            }
            chains.push_back(chain);

		}//end i
		
		//approximate
		float error=approximate(chains, new_approx);
		approx=new_approx;
		if(error<tau) break;
	}
	while(true);
}

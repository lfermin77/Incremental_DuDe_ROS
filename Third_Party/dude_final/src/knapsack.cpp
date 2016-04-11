#include "knapsack.h"
#include "bpc.h"
#include <map>
#include <cfloat>
using namespace std;

#define DEBUG 0

c_knapsack::c_knapsack(vector<c_cutset>& all_cutsets)
{
    m_all_cutsets=all_cutsets;
    m_N=m_M=0;
}

//
// build the knapsack table
//
void c_knapsack::build()
{
	uint size=0;
	for(vector<c_cutset>::iterator i=m_all_cutsets.begin();i!=m_all_cutsets.end();i++){
		size+=i->getDiagonals().size();
	}
	
	build(size);
}

//
// build the knapsack table with size n
//
void c_knapsack::build(uint n)
{
    m_N=n+1; //from 0 to n
    m_M=m_all_cutsets.size();
    m_table.clear();
    initTable();

    for(uint n=1;n<m_N;n++){ //start from capacity==1

#if DEBUG
        cout<<"capacity="<<n<<endl;
#endif

        bool stablized=true;
        uint iteration=0;
        do{
            stablized=true;
            for(uint m=0;m<m_M;m++){
                cell& c=m_table[n][m];
                double old_score=c.score;
                updateCell(n,m,iteration);
                double new_score=c.score;
                if(old_score!=new_score) stablized=false;
            }
            iteration++;
        }
        while(!stablized || iteration<3);
        
        if(n>=3){
        	//if there is no improvement over three consecutive runs
        	//break early since every cutset has at most two cuts
        	if(m_table[n][m_M-1].score==m_table[n-1][m_M-1].score && 
        	   m_table[n][m_M-1].score==m_table[n-2][m_M-1].score )
        	{
        	    m_table.erase(m_table.begin()+n+1,m_table.end());
        	    break;
        	}
        }

#if DEBUG
        cout<<endl;
#endif

    }
}

//get the number of cutsets that is not marked for deletion
inline int count_valid_cutsets(const vector<int>& cset_vector, const vector<char>& flags)
{
    int cset_vector_size=cset_vector.size();
    int reminaing_cutset_count=0;

    for(int j=0;j<cset_vector_size;j++)
    {
        int cset_id=cset_vector[j];
        if(flags[cset_id]=='2') continue; //already removed....
        reminaing_cutset_count++; //
    }//end for j

    return reminaing_cutset_count;
}

//get cutsets that has not been marked
inline void get_remaining_cutsets(const vector<int>& cset_vector, const vector<char>& flags, vector<int>& remains)
{
    int cset_vector_size=cset_vector.size();

    for(int j=0;j<cset_vector_size;j++)
    {
        int cset_id=cset_vector[j];
        if(flags[cset_id]=='2') continue; //already removed....
        if(flags[cset_id]=='1') continue; //can't remove this...
        remains.push_back(cset_id);
    }//end for j
}

inline int find_best_cutsets(const vector<c_cutset>& cutsets)
{
    int v_cutsets_size=cutsets.size();
    double best_score=-FLT_MAX;
    int best_cutset_id=-1;
    for(int c=0;c<v_cutsets_size;c++)
    {
        if(cutsets[c].getWeight()>best_score)
        {
            best_score=cutsets[c].getWeight();
            best_cutset_id=c;
        }
    }

    return best_cutset_id;
}

inline int find_best_cutsets(const vector<c_cutset>& cutsets, const vector<int>& ids)
{
    float highest_score=-FLT_MAX;
    int best_cutset=-1;
    int idsize=ids.size();
    for(int j=0;j<idsize;j++)
    {
        int cset_id=ids[j];
        if( cutsets[cset_id].getWeight()>highest_score)
        {
            highest_score=cutsets[cset_id].getWeight();
            best_cutset=cset_id;
        }
    }
    return best_cutset;
}

//get a list of vertex that can be resolved by this cset
inline void collect_resolvable_vertex(const c_cutset& cset, list<ply_vertex*>& resolv_v, double tau)
{
    resolv_v.push_back(cset.getV());

    switch(cset.getType())
    {
        case c_cutset::SINGLE_PM:
            if(cset.getResolveSize()>1)
            {
                ply_vertex* ov=cset.getDiagonals().front().getOther(cset.getV());
                resolv_v.push_back(ov); //this cutset also resolves ov
            }
            break;
        case c_cutset::SINGLE_TIP: break; //do nothing...
        case c_cutset::DOUBLE_PM_PM:
            if(cset.getResolveSize()>1)
            {
                const c_diagonal& d1=cset.getDiagonals().front();
                const c_diagonal& d2=cset.getDiagonals().back();
                ply_vertex* ov1=d1.getOther(cset.getV());
                ply_vertex* ov2=d2.getOther(cset.getV());

                if(d1.is_resolving(ov1,tau)) resolv_v.push_back(ov1); //this cutset also resolves ov
                if(d2.is_resolving(ov2,tau)) resolv_v.push_back(ov2); //this cutset also resolves ov
            }
            break;
        case c_cutset::DOUBLE_PM_TIP:
            if(cset.getResolveSize()>1)
            {
                const c_diagonal& d1=cset.getDiagonals().front();
                const c_diagonal& d2=cset.getDiagonals().back();
                ply_vertex* ov1=d1.getOther(cset.getV());
                ply_vertex* ov2=d2.getOther(cset.getV());

                if( ov1->getExtra().isPM()  && d1.is_resolving(ov1,tau))
                    resolv_v.push_back(ov1); //this cutset also resolves ov
                if( ov2->getExtra().isPM()  && d2.is_resolving(ov2,tau))
                    resolv_v.push_back(ov2); //this cutset also resolves ov
            }
            break;
        case c_cutset::DOUBLE_TIP_TIP: break; //do nothing
    }
}

//check if a cutset can be removed
inline bool can_be_removed
(int cset_id, const vector<c_cutset>& cutsets,
 map<ply_vertex*, vector<int> > &map_v2cutsets,
 const vector<char>& flags, double tau)
{
    const c_cutset& cset=cutsets[cset_id];
    list<ply_vertex*> resolv_v;
    collect_resolvable_vertex(cset,resolv_v,tau);

    //check all vertieces that this cutset can remove
    for(list<ply_vertex*>::iterator i=resolv_v.begin();i!=resolv_v.end();i++)
    {
        ply_vertex * v=*i;
        const vector<int>& ov_csets=map_v2cutsets[v];
        int validsize=count_valid_cutsets(ov_csets,flags);
        if(validsize<2) return false;
    }

    return true;
}

//storoe cuts in "cuts" and return the score
double c_knapsack::getOptimalCuts(list<ply_vertex*>& pms, double tau, vector<c_diagonal>& cuts)
{
    if(m_table.empty()) return 0;
    if(m_table.back().empty()) return 0;

    map<ply_vertex*, vector<int> > map_v2cutsets;
    cell& c=m_table.back().back();

    vector<c_cutset> all_cutsets;


    //make sure that cutsets are unique...
    {
        int c_cutsets_size=c.cutsets.size();
        for(int i=0;i<c_cutsets_size;i++)
        {
            bool found=false;
            for(int j=i+1;j<c_cutsets_size;j++)
            {
                if(c.cutsets[i]==c.cutsets[j]){
                    found=true;
                    break;
                }
            }//end j

            if(!found) all_cutsets.push_back(c.cutsets[i]);
        }//end i
    }

    //
    //register cutsets to vertex
    //
    int cutset_size=all_cutsets.size();
    for(int i=0;i<cutset_size;i++)
    {
        c_cutset& cset=all_cutsets[i];

        //collect vertices that can be resolved by this cutset
        list<ply_vertex*> resolv_v;
        collect_resolvable_vertex(cset,resolv_v,tau);

        //mark them
        for(list<ply_vertex*>::iterator j=resolv_v.begin();j!=resolv_v.end();j++)
        {
            map_v2cutsets[*j].push_back(i);
        }
    }

    //
    vector<char> book_keeping(cutset_size,0); //flag '1' if the cutset cannot be removed
                                              //flag '2' if the cutset must be removed
    list<c_cutset> missing_cutsets;

    //
    // go through each vertex
    typedef list<ply_vertex*>::iterator VIT;

    //determine cutset that cannot be removed
    for(VIT i=pms.begin();i!=pms.end();i++)
    {
        vector<int>& cset_vector=map_v2cutsets[*i];
        if(cset_vector.size()==1) book_keeping[cset_vector.front()]='1';
    }

    //now check if the cutsets can be removed....
    for(VIT i=pms.begin();i!=pms.end();i++)
    {
        ply_vertex * v=*i;

        vector<int>& cset_vector=map_v2cutsets[v];
        int cset_vector_size=cset_vector.size();
        if(cset_vector_size==0) //nothing resolves this PM....
        {
            //add the best cutset for this vertex....
            const vector<c_cutset>& v_cutsets=v->getExtra().cutsets;
            int best_cutset_id=find_best_cutsets(v_cutsets);
            //add the cutset to missing_cutsets
            if(best_cutset_id>=0) missing_cutsets.push_back(v_cutsets[best_cutset_id]);
        }
        else if(cset_vector_size>1) //there are more than 1 cutsets resolving this vertex
        {
            //remove redundant cuts....
            int reminaing_cutset_count=count_valid_cutsets(cset_vector,book_keeping);
            assert(reminaing_cutset_count); //we must still have some cutset resolving this vertex

            //to be removed...
            vector<int> removal_candidates;
            get_remaining_cutsets(cset_vector,book_keeping,removal_candidates);
            int removal_candidates_size=removal_candidates.size();

            if(reminaing_cutset_count>1 && removal_candidates_size>0)
            {

                if(reminaing_cutset_count>removal_candidates_size) //remove all cuts in removal_candidates
                {
                    for(int j=0;j<removal_candidates_size;j++)
                    {
                        int csetid=removal_candidates[j];

                        if( can_be_removed(csetid,all_cutsets,map_v2cutsets,book_keeping,tau) )
                        {
                            book_keeping[csetid]='2'; //marked for removal
#if DEBUG
                            cout<<"- Remove cutset: id="<<all_cutsets[csetid].getV()->getVID()<<": "<<all_cutsets[csetid]<<endl;
#endif
                        }
                    }
                }
                else //keep one cut and remove all the others
                {
                    //find the best cutset in the removal candidates...
                    int best_cutset=find_best_cutsets(all_cutsets,removal_candidates);

                    //
                    for(int j=0;j<removal_candidates_size;j++)
                    {
                        int cset_id=removal_candidates[j];
                        if(best_cutset==cset_id)
                        {
                            book_keeping[cset_id]='1'; //must keep
                        }
                        else
                        {
                            if( can_be_removed(cset_id,all_cutsets,map_v2cutsets,book_keeping,tau) )
                            {
                                book_keeping[cset_id]='2'; //marked for removal
#if DEBUG
                                cout<<"- Remove cutset: id="<<all_cutsets[csetid].getV()->getVID()<<": "<<all_cutsets[csetid]<<endl;
#endif
                            }
                        }
                    }//end for j
                }

            }
        }
    }//end for i

    //collect all cutsets that needs to be kept
    vector<c_cutset> final_cutsets;
    for(int i=0;i<cutset_size;i++)
    {
        if(book_keeping[i]!='1') continue; //no need to keep
        final_cutsets.push_back(all_cutsets[i]);
    }

    //add missing cutsets
    final_cutsets.insert(final_cutsets.end(),missing_cutsets.begin(),missing_cutsets.end());

    //replace c's cutsets...
    c.cutsets.swap(final_cutsets);

    //
    //
    //
    c.unionCuts(cuts);

#if DEBUG
    uint size=cuts.size();
    cout<<"- final cuts:"<<endl;
    for(uint i=0;i<size;i++)
        cout<<"\t "<<cuts[i].getV1()->getVID()<<", "<<cuts[i].getV2()->getVID()<<endl;
#endif

    return c.score;
}

void c_knapsack::updateCell(uint n, uint m, uint iteration) //update the table cell (n,m)
{
    c_cutset& cset=m_all_cutsets[m];
    uint csetsize=cset.getDiagonals().size();

    if(m==0 && iteration==0){ //first iteration, first cell, bootstrap this row
        cell& c=m_table[n][0];
        bool fit=(n>=csetsize); //fit
        if(fit){
            add2Cell(c,cset);

#if DEBUG
            cout<<"m==0: add cutset"<<cset<<endl;
#endif

        }

        return;
    }

    //previous cutsets
    uint m_1=(m>0)?m-1:m_M-1;
    int min_k=n-csetsize; //the smallest capacity that we should check
    if(min_k<0) min_k=0;

    double max_score=-FLT_MAX;
    cell best_cell;

    //check if we can add cset
    for(int k=n; k>=min_k; k--){ //k is capacity, goes from n to min_k

        cell& c=m_table[k][m_1];
        bool fit=enoughCapacity(n,c,cset); //max capacity is n
        if(fit){
            cell tmp=c;
            add2Cell(tmp,cset);
            if(tmp.score>max_score){
                max_score=tmp.score;
                best_cell=tmp;
            }
        }
    }//end for k

    //check if it's better without adding cset
    cell& c=m_table[n][m_1];
    if(c.score>max_score){
        best_cell=c;

#if DEBUG
        cout<<"m=="<<m<<": skip cutset"<<cset<<endl;
#endif

    }
#if DEBUG
    else{
        cout<<"m=="<<m<<": add cutset"<<cset<<endl;
    }
#endif

    //now, record the best cell
    m_table[n][m]=best_cell;
}

void c_knapsack::initTable() //initialize the table
{
    //create a m_Nxm_M table
    m_table=vector< vector<cell> >(m_N, vector<cell>(m_M));
    //when there are 0 cut sets to choose from (m=0)
    //there are 0 cuts in optimal solution
}

//check if the given cset can be add to c without exceeding capacity
bool c_knapsack::enoughCapacity(uint capacity, const cell& c, c_cutset& cset)
{
    cell tmp=c;
    tmp.addCutset(cset);
    vector<c_diagonal> diagonals;
    tmp.unionCuts(diagonals);

    if(diagonals.size()>capacity) return false;
    return true;
}

// update c by add cset to it
//
// reject the cutsets in c that conflict with cset
// update the score of the cutsets as the total scores to the cuts
//
void c_knapsack::add2Cell(c_knapsack::cell& c, c_cutset& cset)
{
    //add cset to c
    c.addCutset(cset);
    //update score
    c.computeScore();
}

void c_knapsack::cell::addCutset(c_cutset& cset)
{
    typedef vector<c_cutset>::iterator IT;
    vector<c_cutset> new_cutset_list;
    new_cutset_list.push_back(cset);
    for(IT i=cutsets.begin();i!=cutsets.end();i++){
        if(i->conflict(cset)==false) new_cutset_list.push_back(*i);

#if DEBUG
        else
            cout<<"conflict "<<*i<<" VS "<<cset<<endl;
#endif

    }//end for i
    cutsets.swap(new_cutset_list);
}

void c_knapsack::cell::computeScore()
{
    score=0;
    uint size=cutsets.size();
    for(uint i=0;i<size;i++){
        score+=cutsets[i].getWeight();
    }
}

//compute the unions of all cuts in cutsets
void c_knapsack::cell::unionCuts( vector<c_diagonal>& diagonals )
{
    typedef pair<uint,uint> Vpair;
    map<Vpair,c_diagonal> vpair2diagonal; //mapping vpair to diagonal

    uint size=cutsets.size();
    for(uint i=0;i<size;i++){
        c_cutset& cset=cutsets[i];
        const vector<c_diagonal>& dia=cset.getDiagonals();
        uint diasize=dia.size();
        for(uint j=0;j<diasize;j++){
            const c_diagonal& d=dia[j];
            Vpair vpair(d.getV1()->getVID(),d.getV2()->getVID());
            if(vpair.first>vpair.second) swap(vpair.first,vpair.second);
            if( vpair2diagonal.find(vpair)==vpair2diagonal.end() ){
                vpair2diagonal[vpair]=d;
            }
        }//end j
    }//end i

    //store vpair2diagonal to diagonals
    typedef map<Vpair,c_diagonal>::iterator IT;
    for(IT i=vpair2diagonal.begin();i!=vpair2diagonal.end();i++){
        diagonals.push_back(i->second);
    }
}


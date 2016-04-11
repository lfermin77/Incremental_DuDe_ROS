#include "cutset.h"
#include "polygon.h"
#include "intersection.h"
#include "dude_util.h"
#include "holediag.h"

#include <algorithm>

#define DEBUG 0

#define USE_DIHEDRAL_ANGLE_TO_RESOLVE 0

double c_cutset::m_tau=0; //private member, this is specified through build_cutsets

inline double angleAt(const Point2d& p1, const Point2d& p2, const Point2d& p3)
{//to compute the inner angle at p2
    Vector2d v1 = (p1-p2).normalize();
    Vector2d v2 = (p3-p2);

    double x=v1*v2;
    double y=std::sqrt(v2.normsqr()-x*x);
    double angle=atan2(y,x); //between 0 and PI since y is always positive

    int sign=AreaSign(p1.get(),p2.get(),p3.get());
    if(sign==0) return PI;
    if(sign>0) return angle;

    //(sign<0)
    return PI2-angle;
}

inline void sortDiagonalsbyAngle
(ply_vertex * v, const Point2d& start, const Point2d& end, const vector<c_diagonal>& diagonals,
 vector< pair<double, vector<c_diagonal>::const_iterator > >& angles)
{
    typedef vector<c_diagonal>::const_iterator IT;

    {
        double total_angle=angleAt(start,v->getPos(),end); //dihedral angle of v between start and end

        //
        //start to compute the angles between each diagonal and edge (start,v)
        //
        for(IT i=diagonals.begin();i!=diagonals.end();i++)
        {
            assert(i->getV1()==v || i->getV2()==v);

            //pt is the "other" end point of the diagonal that is not v
            ply_vertex * otherv=i->getOther(v);
            const Point2d& pt=otherv->getPos();

            //angle between this diagonal and and edge (start,v)
            double angle=angleAt(start,v->getPos(),pt);

            //reject diagonals whose angles is out of bound
            if(angle>total_angle) continue;

#if DEBUG
            cout<<"\t ("<<i->getV1()->getVID()<<","<<i->getV2()->getVID()<<") Angle="<<angle<<endl;
#endif

            //add angle and iterator to angle list
            angles.push_back(pair<double,IT>(angle,i));
        }

        //add the edge (v,end) to the angle list
        angles.push_back(pair<double,IT>(total_angle,diagonals.end()));
    }

    //sort angles from small to large
    sort(angles.begin(),angles.end());

    //make sure that the angle between (start,v) and (v,end) is at the end of the list
    assert(angles.back().second==diagonals.end());
}

//get angles between two adjacent cuts
inline void getDihedralAngles(vector<double>& dihedral_angles, const vector< pair<double,vector<c_diagonal>::const_iterator> >& angles)
{
    int size = angles.size();
    dihedral_angles.reserve(size);
    dihedral_angles.push_back(angles[0].first);
    for(int i=0; i<size-1; i++)
        dihedral_angles.push_back( angles[i+1].first-angles[i].first );
}

//locate the position where accumulative angles become larger than PI
inline int locateReflex
(vector<double> dihedral_angles, int start, int end, bool begin, double tau=PI)
{
    double cumulate_angle=0;
    int position = start;
    while (true){
        cumulate_angle+=dihedral_angles[position];
        if(cumulate_angle>tau)
            return position;
        if(begin){
            position++;
            if(position>end)
                break;
        }
        else {
            position--;
            if(position<end)
                break;
        }
    }
    return end;
}


#if USE_DIHEDRAL_ANGLE_TO_RESOLVE

//a static method for buidling all cutsets for a given vertex v
//assuming that the diagonals are bounded between segments (v,start) and (v,end)
void c_cutset::build_cutsets
(ply_vertex * v, const Point2d& start, const Point2d& end,
 const vector<c_diagonal>& diagonals, double tau,
 vector<c_cutset>& cutset_list)
{

    m_tau=tau;

#if DEBUG
    if(diagonals.empty()){
        cerr<<"! ERROR: no diagonals for building cutsets for PM["<<v->getVID()<<"]"<<endl;
    }
    cout<<"- PM["<<v->getVID()<<"] has "<<diagonals.size()<<" diagonals"<<endl;
#endif

    typedef vector<c_diagonal>::const_iterator IT;

    //category of tips
    list<c_cutset> single_PM;      //single diagonal, connecting v to a PM (pocket minimum)
    list<c_cutset> single_TIP;     //single diagonal, connecting v to a tip (regular vertex)
    list<c_cutset> double_PM_PM;   //double diagonal, connecting v to two PMs
    list<c_cutset> double_PM_TIP;  //double diagonal, connecting v to a PM and a tip
    list<c_cutset> double_TIP_TIP; //double diagonal, connecting v to two tips

    //start to compute angles
    vector< pair<double,IT> > angles;
    sortDiagonalsbyAngle(v,start,end,diagonals,angles);

    //compute dihedral angles (angles between diagonals)
    vector<double> dihedral_angles;
    getDihedralAngles(dihedral_angles,angles);

    // (type 1) single diagonal
    //
    // collect all cuts that can resolve this pocket minimum by itself
    // locate the position from 0 where dihedral angle becomes larger than 180
    // locate the position from size where dihedral angle becomes larger than 180

    uint diagonal_size=angles.size()-1; //minus one because the last angle is (v,end)
    int pos1 = locateReflex(dihedral_angles,diagonal_size,0,false, PI*1.1    );//cuts [0 1 ... pos1-1] will not resolve this minimum
    int pos2 = locateReflex(dihedral_angles,0,diagonal_size,true, PI*1.1); //cuts [pos2 ... size] will not resolve this minimum

#if DEBUG
    cout<<"- pos1="<<pos1<<endl;
    cout<<"- pos2="<<pos2<<endl;
#endif

    // collect all cut sets [{pos1} ... {pos2-1}] because each cut resolves this minimum by itself
    // classify these diagonals to single_PM and single_PM
    for(int pos=pos1; pos<pos2; pos++){
        IT it=angles[pos].second;
        c_cutset cset;//cut set with one single cut
        cset.m_v=v;
        cset.m_diagonals.push_back(*it);

        if(it->getOther(v)->getExtra().isPM()){
            cset.setType(c_cutset::SINGLE_PM);
            single_PM.push_back(cset);
        }
        else{
            cset.setType(c_cutset::SINGLE_TIP);
            single_TIP.push_back(cset);
        }

    }//end for

    // (type 2) double diagonals
    //
    //to collect cut sets with size 2, there are no cut sets with size larger than 2
    //[0 1 ... pos1-2] [pos2+1 ... end-1]
    //compute angles between pos2 and 0 and stop at angle larger than 180 which is pos3
    //if pos3==pos1-1, return no other cut sets
    //else return {pos3+1,pos2}, {pos3+2,pos2}...{pos1-1,pos2}
    //do the same thing to [pos2+1 ... size] may use some intermediate results for pos2+1
    //if(pos1>pos2-1) return;

    int pos=pos2;
    while (pos<(int)diagonal_size)
    {

        assert(angles[pos].second!=diagonals.end());

        //diagonal iterator for pos
        IT dia_it1=angles[pos].second;
        ply_vertex * other_v1=dia_it1->getOther(v);

        //
        int pos3 = locateReflex(dihedral_angles,pos,0,false, PI*1.25);

#if DEBUG
    cout<<"- pos3="<<pos3<<endl;
#endif

        //if(pos3==pos1-1) break;
        for(int k=pos3; k<pos1; k++)
        {
            //diagonal iterator for k
            IT dia_it2=angles[k].second;
            ply_vertex * other_v2=dia_it2->getOther(v);

            c_cutset cset;//cut set with two cuts
            cset.m_v=v;
            assert(angles[k].second!=diagonals.end());
            cset.m_diagonals.push_back(*dia_it1);
            cset.m_diagonals.push_back(*dia_it2);

            //connecting two pm
            if(other_v1->getExtra().isPM() && other_v2->getExtra().isPM()){
                cset.setType(c_cutset::DOUBLE_PM_PM);
                double_PM_PM.push_back(cset);
            }
            //connecting one pm and one tip
            else if(other_v1->getExtra().isPM() || other_v2->getExtra().isPM()){
                cset.setType(c_cutset::DOUBLE_PM_TIP);
                double_PM_TIP.push_back(cset);
            }
            //connecting two tips
            else{
                cset.setType(c_cutset::DOUBLE_TIP_TIP);
                double_TIP_TIP.push_back(cset);
            }
        }
        pos++;
    }

    handle_single_PM(single_PM);
    handle_single_TIP(single_TIP);
    handle_double_PM_PM(double_PM_PM);
    handle_double_PM_TIP(double_PM_TIP);
    handle_double_TIP_TIP(double_TIP_TIP);

    //now, if there is no cutset of this vertex (because no diagonals can resolve this PM)
    //we simply select one that has the highest score
    //TODO: we can also create a cut that bisects this vertex...
    if(v->getExtra().cutsets.empty()){
        double best_score=-FLT_MAX;
        c_cutset best_cutset;

        for(uint i=0;i<diagonal_size;i++){
            IT it=angles[i].second;
            c_cutset cset;
            cset.m_v=v;
            cset.m_diagonals.push_back(*it);

            if(it->getOther(v)->getExtra().isPM()){
                cset.setType(c_cutset::SINGLE_PM);
                cset.analyze_single_PM();
            }
            else{
                cset.setType(c_cutset::SINGLE_TIP);
                cset.analyze_single_TIP();
            }

            cset.setResolveSize(cset.getResolveSize()-1);
            double score=cset.getWeight();
            if(score>best_score){
                best_score=score;
                best_cutset=cset;
            }
        }//end i

        if(best_score!=-FLT_MAX)
            v->getExtra().cutsets.push_back(best_cutset);
    }
}

#else //USE_DIHEDRAL_ANGLE_TO_RESOLVE==0

//a static method for buidling all cutsets for a given vertex v  using residual concavity
//assuming that the diagonals are bounded between segments (v,start) and (v,end)
void c_cutset::build_cutsets
     (ply_vertex * v, const Point2d& start, const Point2d& end,
      const vector<c_diagonal>& diagonals, double tau,
      vector<c_cutset>& cutset_list)
 //cutset_list stores results
{
    m_tau=tau;

#if DEBUG
    if(diagonals.empty()){
        cerr<<"! ERROR: no diagonals for building cutsets for PM["<<v->getVID()<<"]"<<endl;
    }
    cout<<"- PM["<<v->getVID()<<"] has "<<diagonals.size()<<" diagonals"<<endl;
#endif

    typedef vector<c_diagonal>::const_iterator IT;

    double total_angle=angleAt(start,v->getPos(),end); //dihedral angle of v between start and end

    //category of tips
    list<c_cutset> single_PM;      //single diagonal, connecting v to a PM (pocket minimum)
    list<c_cutset> single_TIP;     //single diagonal, connecting v to a tip (regular vertex)
    list<c_cutset> double_PM_PM;   //double diagonal, connecting v to two PMs
    list<c_cutset> double_PM_TIP;  //double diagonal, connecting v to a PM and a tip
    list<c_cutset> double_TIP_TIP; //double diagonal, connecting v to two tips

    //start to compute angles
    vector< pair<double,IT> > angles;
    sortDiagonalsbyAngle(v,start,end,diagonals,angles);

    //compute dihedral angles (angles between diagonals)
//    vector<double> dihedral_angles;
//    getDihedralAngles(dihedral_angles,angles);

    // (type 1) single diagonal
    //
    // collect all cuts that can resolve this pocket minimum by itself
    //
    // go through all diagonals and determine it the residual concavity is too high
    // classify these diagonals to single_PM and single_PM
    //
    uint diagonal_size=angles.size()-1;
    vector<bool> single_resolved(diagonal_size,false);

    for(uint pos=0; pos<diagonal_size; pos++)
    {
        IT it=angles[pos].second;

        if( it->is_resolving(v,m_tau)==false ) continue; //this diagonal does not resolve v...

        const ply_vertex* v1 = (*it).getV1();
        const ply_vertex* v2 = (*it).getV2();
        if(!isHoleDiagonalValid(v1,v2)) // check if both of two end points of this diagonal is in hole and they are approximated by a single line
        	continue;


        //remember that this diagonal resolves v by itself
        single_resolved[pos]=true;

        c_cutset cset;//cut set with one single cut
        cset.m_v=v;
        cset.m_diagonals.push_back(*it);

        if(it->getOther(v)->getExtra().isPM()){
            cset.setType(c_cutset::SINGLE_PM);
            single_PM.push_back(cset);
        }
        else{
            cset.setType(c_cutset::SINGLE_TIP);
            single_TIP.push_back(cset);
        }

    }//end for

    // (type 2) double diagonals
    //
    //to collect cut sets with size 2, there are no cut sets with size larger than 2

    for(uint pos1=0; pos1<diagonal_size; pos1++)
    {
        if(single_resolved[pos1]) continue;
        const c_diagonal& d1=*angles[pos1].second;
        ply_vertex * other_v1=d1.getOther(v);

        for(uint pos2=pos1+1; pos2<diagonal_size; pos2++)
        {
            if(single_resolved[pos2]) continue;
            const c_diagonal& d2=*angles[pos2].second;
            ply_vertex * other_v2=d2.getOther(v);

            //check if d1 and d2 together can resolve v
            if( d1.is_resolving(d2,v,m_tau)==false ) continue; //cannot resolve v...

            //
            //create cut set for d1 and d2
            //
            c_cutset cset;//cut set with two cuts
            cset.m_v=v;
            cset.m_diagonals.push_back(d1);
            cset.m_diagonals.push_back(d2);

            //connecting two pm
            if(other_v1->getExtra().isPM() && other_v2->getExtra().isPM()){
                cset.setType(c_cutset::DOUBLE_PM_PM);
                double_PM_PM.push_back(cset);
            }
            //connecting one pm and one tip
            else if(other_v1->getExtra().isPM() || other_v2->getExtra().isPM()){
                cset.setType(c_cutset::DOUBLE_PM_TIP);
                double_PM_TIP.push_back(cset);
            }
            //connecting two tips
            else{
                cset.setType(c_cutset::DOUBLE_TIP_TIP);
                double_TIP_TIP.push_back(cset);
            }

        }//end pos2

    }//end pos1

    //[0 1 ... pos1-2] [pos2+1 ... end-1]
    //compute angles between pos2 and 0 and stop at angle larger than 180 which is pos3
    //if pos3==pos1-1, return no other cut sets
    //else return {pos3+1,pos2}, {pos3+2,pos2}...{pos1-1,pos2}
    //do the same thing to [pos2+1 ... size] may use some intermediate results for pos2+1
    //if(pos1>pos2-1) return;

    /*
    int pos=pos2;
    while (pos<(int)diagonal_size)
    {

        assert(angles[pos].second!=diagonals.end());

        //diagonal iterator for pos
        IT dia_it1=angles[pos].second;
        ply_vertex * other_v1=dia_it1->getOther(v);

        //
        int pos3 = locateReflex(dihedral_angles,pos,0,false, PI*1.25);

#if DEBUG
    cout<<"- pos3="<<pos3<<endl;
#endif

        //if(pos3==pos1-1) break;
        for(int k=pos3; k<pos1; k++)
        {
            //diagonal iterator for k
            IT dia_it2=angles[k].second;
            ply_vertex * other_v2=dia_it2->getOther(v);

            c_cutset cset;//cut set with two cuts
            cset.m_v=v;
            assert(angles[k].second!=diagonals.end());
            cset.m_diagonals.push_back(*dia_it1);
            cset.m_diagonals.push_back(*dia_it2);

            //connecting two pm
            if(other_v1->getExtra().isPM() && other_v2->getExtra().isPM()){
                cset.setType(c_cutset::DOUBLE_PM_PM);
                double_PM_PM.push_back(cset);
            }
            //connecting one pm and one tip
            else if(other_v1->getExtra().isPM() || other_v2->getExtra().isPM()){
                cset.setType(c_cutset::DOUBLE_PM_TIP);
                double_PM_TIP.push_back(cset);
            }
            //connecting two tips
            else{
                cset.setType(c_cutset::DOUBLE_TIP_TIP);
                double_TIP_TIP.push_back(cset);
            }
        }
        pos++;
    }
    */

    handle_single_PM(single_PM);
    handle_single_TIP(single_TIP);
    handle_double_PM_PM(double_PM_PM);
    handle_double_PM_TIP(double_PM_TIP);
    handle_double_TIP_TIP(double_TIP_TIP);

    //now, if there is no cutset of this vertex (because no diagonals can resolve this PM)
    //we simply select one that has the highest score
    //TODO: we can also create a cut that bisects this vertex...
    if(v->getExtra().cutsets.empty()){
        double best_score=-FLT_MAX;
        c_cutset best_cutset;

        for(uint i=0;i<diagonal_size;i++){
            IT it=angles[i].second;
            c_cutset cset;
            cset.m_v=v;
            cset.m_diagonals.push_back(*it);

            if(it->getOther(v)->getExtra().isPM()){
                cset.setType(c_cutset::SINGLE_PM);
                cset.analyze_single_PM();
            }
            else{
                cset.setType(c_cutset::SINGLE_TIP);
                cset.analyze_single_TIP();
            }

            cset.setResolveSize(cset.getResolveSize()-1);
            double score=cset.getWeight();
            if(score>best_score){
                best_score=score;
                best_cutset=cset;
            }
        }//end i

        if(best_score!=-FLT_MAX)
            v->getExtra().cutsets.push_back(best_cutset);
    }
}

#endif //USE_DIHEDRAL_ANGLE_TO_RESOLVE



//keep all
void c_cutset::handle_single_PM(list<c_cutset>& cutsets)
{
    typedef list<c_cutset>::iterator IT;

    //basically, we should get the shortest tip tip
    //this will be done by checking the diagonal weight
    for(IT i=cutsets.begin();i!=cutsets.end();i++)
    {
        i->analyze_single_PM();
        i->getV()->getExtra().cutsets.push_back(*i);

#if DEBUG
            cout<<*i<<" [single pm]"<<endl;
#endif

    }//end i
}

//keep only one best single_tip
void c_cutset::handle_single_TIP(list<c_cutset>& cutsets)
{
    typedef list<c_cutset>::iterator IT;

    //basically, we should get the shortest single tip
    //this will be done by checking the diagonal weight
    double shortest_resolve=FLT_MAX;
    IT shortest_cutset_resolve=cutsets.end();
    double shortest_none_resolve=FLT_MAX;
    IT shortest_cutset_none_resolve=cutsets.end();

    for(IT i=cutsets.begin();i!=cutsets.end();i++)
    {
        i->analyze_single_TIP();
        const c_diagonal& dia=i->getDiagonals().front();
#if USE_DIHEDRAL_ANGLE_TO_RESOLVE
        if(dia.is_angle_resolving(i->getV(),PI*1.1)==false)
#else
        if(dia.is_resolving(i->getV(),m_tau)==false ) // || dia.is_angle_resolving(i->getV())==false)
#endif
        {
            if(dia.getLength()<shortest_none_resolve)
            {
                shortest_none_resolve=dia.getLength();
                shortest_cutset_none_resolve=i;
            }
        }
        else{

            if(dia.getLength()<shortest_resolve)
            {
                shortest_resolve=dia.getLength();
                shortest_cutset_resolve=i;
            }
        }
    }//end i

    if(shortest_cutset_resolve!=cutsets.end()){
        shortest_cutset_resolve->getV()->getExtra().cutsets.push_back(*shortest_cutset_resolve);
#if DEBUG
        cout<<*shortest_cutset_resolve<<" [single tip]"<<endl;
#endif
    }
    else if(shortest_cutset_none_resolve!=cutsets.end()){
        shortest_cutset_none_resolve->getV()->getExtra().cutsets.push_back(*shortest_cutset_none_resolve);
#if DEBUG
        cout<<*shortest_cutset_none_resolve<<" [single tip]"<<endl;
#endif
    }

}



//keep all
void c_cutset::handle_double_PM_PM(list<c_cutset>& cutsets)
{
    typedef list<c_cutset>::iterator IT;

    //basically, we should get the shortest tip tip
    //this will be done by checking the diagonal weight
    for(IT i=cutsets.begin();i!=cutsets.end();i++)
    {
        i->analyze_double_PM_PM();
        i->getV()->getExtra().cutsets.push_back(*i);

#if DEBUG
            cout<<*i<<" [double pm pm]"<<endl;
#endif

    }//end i
}


void c_cutset::handle_double_PM_TIP(list<c_cutset>& cutsets)
{
    typedef list<c_cutset>::iterator IT;

    //basically, we should get the shortest tip tip
    double shortest=FLT_MAX;
    IT best_cutset=cutsets.end();

    for(IT i=cutsets.begin();i!=cutsets.end();i++)
    {
        double rs=i->analyze_double_PM_TIP(); //resolve size

        if(rs==2){
            i->getV()->getExtra().cutsets.push_back(*i);

#if DEBUG
            cout<<*i<<" [double tip pm] (1)"<<endl;
#endif

        }
        else{
            const c_diagonal& dia1=i->getDiagonals().front();
            const c_diagonal& dia2=i->getDiagonals().back();
            double len=dia1.getLength()+dia2.getLength();
            if(len<shortest)
            {
                shortest=len;
                best_cutset=i;
            }
        }
    }//end i

    if(best_cutset!=cutsets.end()){
        best_cutset->getV()->getExtra().cutsets.push_back(*best_cutset);

#if DEBUG
        cout<<*best_cutset<<" [double tip pm] (2)"<<endl;
#endif

    }
}


void c_cutset::handle_double_TIP_TIP(list<c_cutset>& cutsets)
{
    typedef list<c_cutset>::iterator IT;

    //basically, we should get the shortest tip tip
    //this will be done by checking the diagonal weight
    double shortest=FLT_MAX;
    IT best_cutset=cutsets.end();
    for(IT i=cutsets.begin();i!=cutsets.end();i++)
    {
        i->analyze_double_TIP_TIP();

        const c_diagonal& dia1=i->getDiagonals().front();
        const c_diagonal& dia2=i->getDiagonals().back();
        double score=dia1.getLength()+dia2.getLength();
        if(score<shortest)
        {
            shortest=score;
            best_cutset=i;
        }
    }//end i

    if(best_cutset!=cutsets.end()){

#if DEBUG
        cout<<*best_cutset<<" [double tip tip]"<<endl;
#endif
        best_cutset->getV()->getExtra().cutsets.push_back(*best_cutset);
    }
}

short c_cutset::analyze_single_PM()
{
    ply_vertex * v=getV();
    const c_diagonal& dia=getDiagonals().front();

    ply_vertex * o=dia.getOther(v);
    //we know that o is PM, check if dia can resolve PM
#if USE_DIHEDRAL_ANGLE_TO_RESOLVE
    if(dia.is_angle_resolving(o,PI*1.1)){
#else
    if(dia.is_resolving(o,m_tau)){
#endif
        setResolveSize(2); //this cutset can resolve 2 PMs
    }
    else{
        setResolveSize(1); //this cutset can resolve 1 PM, i.e., v itself
    }

    return getResolveSize();
}

short c_cutset::analyze_single_TIP()
{
    setResolveSize(1);
    return 1;
}

short c_cutset::analyze_double_PM_TIP()
{
    ply_vertex * v=getV();
    const c_diagonal& dia1=getDiagonals().front();
    const c_diagonal& dia2=getDiagonals().back();

    ply_vertex * o1=dia1.getOther(v);
    ply_vertex * o2=dia2.getOther(v);

    if(o1->getExtra().isPM())
    {
#if USE_DIHEDRAL_ANGLE_TO_RESOLVE
        if(dia1.is_angle_resolving(o1,PI*1.1))
#else
        if(dia1.is_resolving(o1,m_tau))
#endif
        { setResolveSize(2); }
        else{ setResolveSize(1); }
    }
    else{ //o2 is PM
        assert(o2->getExtra().isPM());

#if USE_DIHEDRAL_ANGLE_TO_RESOLVE
        if(dia2.is_angle_resolving(o2,PI*1.1))
#else
        if(dia2.is_resolving(o2,m_tau))
#endif
        { setResolveSize(2); }
        else{ setResolveSize(1); }
    }

    return getResolveSize();
}


short c_cutset::analyze_double_PM_PM()
{
    ply_vertex * v=getV();
    const c_diagonal& dia1=getDiagonals().front();
    const c_diagonal& dia2=getDiagonals().back();

    ply_vertex * o1=dia1.getOther(v);
    ply_vertex * o2=dia2.getOther(v);

#if USE_DIHEDRAL_ANGLE_TO_RESOLVE
    bool r1=dia1.is_angle_resolving(o1,PI*1.1);
    bool r2=dia2.is_angle_resolving(o2,PI*1.1);
#else
    bool r1=dia1.is_resolving(o1,m_tau);
    bool r2=dia2.is_resolving(o2,m_tau);
#endif

    if(r1&&r2) setResolveSize(3);
    else if(r1||r2) setResolveSize(2);
    else setResolveSize(1);

    return getResolveSize();
}

short c_cutset::analyze_double_TIP_TIP()
{
    setResolveSize(1);
    return 1;
}

double c_cutset::computeWeight()
{

    c_diagonal& d1=m_diagonals.front();
    double my_concavity=m_v->getExtra().concavity;

    if(m_resolves_n_PM==3)
    {
        c_diagonal& d2=m_diagonals.back();
        ply_vertex * o1=d1.getOther(m_v);
        ply_vertex * o2=d2.getOther(m_v);
        double w1=(my_concavity+o1->getExtra().concavity);
        double w2=(my_concavity+o2->getExtra().concavity);
        return (w1+w2)/(d1.getLength()+d2.getLength());
    }

    if(m_resolves_n_PM==2)
    {

        if(m_diagonals.size()==1)
        {
            ply_vertex * o1=d1.getOther(m_v);
            return (my_concavity+o1->getExtra().concavity)/d1.getLength();
        }
        else{
            c_diagonal& d2=m_diagonals.back();
            ply_vertex * o1=d1.getOther(m_v);
            ply_vertex * o2=d2.getOther(m_v);
            double w=my_concavity;

#if USE_DIHEDRAL_ANGLE_TO_RESOLVE
            if(o1->getExtra().isPM() && d1.is_angle_resolving(o1,PI*1.1))
#else
            if(o1->getExtra().isPM() && d1.is_resolving(o1,m_tau))
#endif//USE_DIHEDRAL_ANGLE_TO_RESOLVE
            {//d1 resovles o1
                w+=o1->getExtra().concavity;
                if(m_type==DOUBLE_PM_PM) w+=(o2->getExtra().concavity/2);
            }
            else{ //d2 must resolves o2
                w+=o2->getExtra().concavity;
                if(m_type==DOUBLE_PM_PM) w+=(o1->getExtra().concavity/2);
            }


            return w/(d1.getLength()+d2.getLength());
        }
    }

    if(m_resolves_n_PM==1)
    {

        if(m_diagonals.size()==1){ //
            double w=my_concavity;

            if(m_type==SINGLE_PM){ //award connection between two PMs
                ply_vertex * o1=d1.getOther(m_v);
                w+=(o1->getExtra().concavity/2);
            }

            return w/d1.getLength();
        }
        else
        {
            double w=my_concavity;
            c_diagonal& d2=m_diagonals.back();

            if(m_type==DOUBLE_PM_PM){ //award connection between PMs
                ply_vertex * o1=d1.getOther(m_v);
                ply_vertex * o2=d2.getOther(m_v);
                w+=(o1->getExtra().concavity/2);
                w+=(o2->getExtra().concavity/2);
            }
            else if(m_type==DOUBLE_PM_TIP){ //award connection between PMs
                ply_vertex * o1=d1.getOther(m_v);
                ply_vertex * o2=d2.getOther(m_v);
                if(o1->getExtra().isPM())
                    w+=(o1->getExtra().concavity/2);
                else if(o2->getExtra().isPM())
                    w+=(o2->getExtra().concavity/2);
            }

            return w/(d1.getLength()+d2.getLength());
        }
    }

    if(m_resolves_n_PM==0)
    {
        double w=my_concavity/2;
        if(m_type==SINGLE_PM){ //award connection between two PMs
            ply_vertex * o1=d1.getOther(m_v);
            w+=(o1->getExtra().concavity/2);
        }
        return w/d1.getLength();
    }

    assert(false); //should not be here...
    return 0;
}

//check if the given cutset conflicts with this cutset
bool c_cutset::conflict(const c_cutset& other) const
{

    if(other.m_v==m_v){
        return true;
    }

    //return false;

    uint othersize=other.getDiagonals().size();
    uint mysize=getDiagonals().size();

    if(othersize==2 && mysize==2){
        return false;
    }


    //if the cutsets contain the same diagonal, return false (no conflict)
    if(othersize==1 && mysize==1){
        if(other.getDiagonals().front()==getDiagonals().front()) return false; //same diagonal
    }

    //if a cutset is a subset of another cutset, return false (no conflict)
    if(othersize==1 && mysize==2){
        const c_diagonal& diagonal=other.getDiagonals().front();
        if(diagonal==getDiagonals().front() || diagonal==getDiagonals().back() )
            return false;
    }

    if(othersize==2 && mysize==1){
        const c_diagonal& diagonal=getDiagonals().front();
        if(diagonal==other.getDiagonals().front())
            return false;
        if(diagonal==other.getDiagonals().back())
            return false;
    }

    //non of above...
    const c_diagonal& diagonal=other.getDiagonals().front();
    if(conflict(other,diagonal)) return true;

    const c_diagonal& diagonal2=getDiagonals().front();
    if(other.conflict(*this,diagonal2)) return true;

    if(othersize==2){
        const c_diagonal& diagonal=other.getDiagonals().back();
        if(conflict(other,diagonal)) return true;
    }

    if(mysize==2){
        const c_diagonal& diagonal=getDiagonals().back();
        if(other.conflict(*this,diagonal)) return true;
    }

    return false;
}

bool c_cutset::conflict(const c_cutset& other, const c_diagonal& diagonal) const
{
    ply_vertex * v=diagonal.getOther(other.m_v);
    if(v==m_v){ //check if other is a cutset for m_v
        if(other.isMemberOf(m_v->getExtra().cutsets)) return true;
    }
    return false;
}

//check if this cutset is a member of the given cutset list
bool c_cutset::isMemberOf(const vector<c_cutset>& cutsets) const
{
    uint size=cutsets.size();
    for(uint i=0;i<size;i++){
        if(cutsets[i]==*this) return true;
    }

    return false;
}

//comparison
bool c_cutset::operator==(const c_cutset& other) const
{
    if(other.m_diagonals.size()!=m_diagonals.size()) return false;

    const c_diagonal& d1=m_diagonals.front();
    const c_diagonal& d2=other.m_diagonals.front();

    if(m_diagonals.size()==1) //one cut
        return d1==d2;

    const c_diagonal& d3=m_diagonals.back();
    const c_diagonal& d4=other.m_diagonals.back();
    if(d1==d2 && d3==d4) return true;
    if(d1==d4 && d3==d2) return true;

    return false;
}

ostream& operator<<(ostream& out, const c_cutset& cset)
{
    const vector<c_diagonal>& diagonals=cset.getDiagonals();
    if(cset.getDiagonals().size()==1)
    {
        out<<"\t cutset {("<<diagonals.back().getV1()->getVID()<<","<<diagonals.back().getV2()->getVID()<<")}";
        out<<" score="<<cset.getWeight();
        out<<" "<<((cset.getType()==c_cutset::SINGLE_PM)?"SINGLE_PM":"SINGLE_TIP");
    }
    else
    {
        out<<"\t cutset {("<<diagonals.front().getV1()->getVID()<<","<<diagonals.front().getV2()->getVID()<<"), "
            <<"("<<diagonals.back().getV1()->getVID()<<","<<diagonals.back().getV2()->getVID()<<")}";
        out<<" score="<<cset.getWeight();
        out<<" ";
        if(cset.getType()==c_cutset::DOUBLE_PM_PM) out<<"DOUBLE_PM_PM";
        else if(cset.getType()==c_cutset::DOUBLE_PM_TIP) out<<"DOUBLE_PM_TIP";
        else if(cset.getType()==c_cutset::DOUBLE_TIP_TIP) out<<"DOUBLE_TIP_TIP";
        else out<<"??";
    }
    out<<" ("<<cset.getResolveSize()<<")";

    return out;
}


void c_cutset::setResolveSize(short n) {
    c_diagonal& d1=m_diagonals.front();
    bool check=false;

//    if(d1.getV1()->getVID()==36 && d1.getV2()->getVID()==21 && m_diagonals.size()==1)
//    {
//        check=true;
//        cout<<"check is true now"<<endl;
//    }


    m_resolves_n_PM=n;

    m_weight=computeWeight();

} //update weight

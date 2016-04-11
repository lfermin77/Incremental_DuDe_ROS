#include "SE2d_skeleton.h"
#include "SE2d_data.h"
#include "SE2d_eigen.h"
#include "polygon.h"

///////////////////////////////////////////////////////////////////////////////
typedef pair<Point2d,Point2d> Line;
typedef list< pair<float,Point2d> > MouthLink;

///////////////////////////////////////////////////////////////////////////////
//
// Simplify links from mouth to principle axis
//
///////////////////////////////////////////////////////////////////////////////

inline void CreateSKG(SKG& g, MouthLink& l, vector<float>& u, Line& PA)
{
    typedef MouthLink::iterator                  MIT;
    typedef vector<float>::const_iterator        UIT;

    //add points and edges on MA
    for(UIT iu=u.begin();iu!=u.end();iu++ ){
        sk_node node;
        //node.m_type=sk_node::On_MA;
        for(int i=0;i<2;i++) 
			node.m_pos[i]=PA.first[i]*(1-*iu)+PA.second[i]*(*iu);
        VID id=g.AddVertex(node);
		if( iu-u.begin()==0 ) continue;
		g.AddEdge(id,id-1,sk_edge(1));
    }

    //add points and edges on mouth
    for( MIT il=l.begin();il!=l.end();il++ ){
        sk_node node;
        //node.m_type=sk_node::On_Mouth;
        node.m_pos=il->second;
        VID id=g.AddVertex(node);

		//construct pt on MA
		float s=il->first;
        sk_node ma_node;
        //ma_node.m_type=sk_node::On_MA;
        for(int i=0;i<2;i++) 
			ma_node.m_pos[i]=PA.first[i]*(1-s)+PA.second[i]*s;
		//make connection
		g.AddEdge(id,g.GetVID(ma_node),sk_edge(1));
    }
}


///////////////////////////////////////////////////////////////////////////////
//
// Simplify links from mouth to principle axis
//
///////////////////////////////////////////////////////////////////////////////

inline void ReAssignU(MouthLink & mouth_link, vector<float>& ma_pt)
{
	typedef vector<float>::const_iterator        UIT;
	typedef MouthLink::iterator                  MIT;

	for(MIT im=mouth_link.begin();im!=mouth_link.end();im++){
		float small_dist=2; float id=0;
		for( UIT iu=ma_pt.begin();iu!=ma_pt.end();iu++ ){
			float dist=(float)fabs(im->first-*iu);
			if( dist<small_dist ){
				small_dist=dist;
				id=*iu;
			}
		}//end UIT iu
		im->first=id;
	}//end MIT im
}

inline void GroupU(Line& PA,vector<float>& ma_pt,double length)
{
	double bound=0.25;//(PA.first-PA.second).norm()/4;
	int size=ma_pt.size()-1;
	if( size>1 ){
		list<float> tmp;
		for( int iu=1;iu<size;iu++ ){
			if( tmp.empty() ) tmp.push_back(ma_pt[iu]);
			else{
				if( (ma_pt[iu]-tmp.back())<bound ) continue; //too small
				tmp.push_back(ma_pt[iu]);
			}
		}
		if( ma_pt.size()==0 ) ma_pt.push_back(ma_pt[(size+1)/2]);
		if( tmp.front()>0.6 ) tmp.push_front(0);
		if( 1-tmp.back()>0.6 ) tmp.push_back(1);
		ma_pt.clear();
		ma_pt.insert(ma_pt.begin(),tmp.begin(),tmp.end());
	}
}

inline double MaxLength
(Line& PA, MouthLink & mouth_link,vector<float>& ma_pt)
{
	typedef MouthLink::iterator MIT;
	double max_len=0;
	ma_pt.push_back(0); ma_pt.push_back(1);
	//compute max dist from mouth to pa
	for(MIT im=mouth_link.begin();im!=mouth_link.end();im++){
		pair<float,Point2d> link=*im;
		Point2d pt;
		const float& u=link.first;
		for(int i=0;i<2;i++) pt[i]=(1-u)*PA.first[i]+u*PA.second[i];
		double len=(pt-link.second).normsqr();
		if( len>max_len ) max_len=len;
		ma_pt.push_back(u);
	}
	sort(ma_pt.begin(),ma_pt.end());
	max_len=sqrtf(max_len);
	return max_len;
}

inline void SimplifyLink
(Line& PA, MouthLink & mouth_link, vector<float>& ma_pt)
{
	typedef MouthLink::iterator MIT;
	//find the max link length
	double max_len=MaxLength(PA,mouth_link,ma_pt);
	//group
	GroupU(PA,ma_pt,max_len);
	//re-assign
	ReAssignU(mouth_link,ma_pt);
}

///////////////////////////////////////////////////////////////////////////////
//
// Connect Mouth to the Principle Axis 
//
///////////////////////////////////////////////////////////////////////////////

inline double ClosestToLine( const Line& l1, const Line& l2 )
{
    Vector2d a=l1.first-l1.second;
    Vector2d b=l2.first-l2.second;
    Vector2d c=l2.first-l1.first;

    double ab=a*b; double bc=b*c; double bb=b*b;
    double ac=a*c; double aa=a*a;

	double base=(aa*bb-ab*ab);
	if( base==0 ) base=1e-10;
    double s=(ab*bc-bb*ac)/base;
    if( s<0 ) s=0; 
	if( s>1 ) s=1;

    return s;
}

inline double ClosestPt
(const Line& l, const Point2d& p, const Vector2d& n)
{
    Line l2(p+(n*100),p+(n*-100));
    
    double s=ClosestToLine(l,l2);
    Point2d result;
	for(int i=0;i<2;i++) result[i]=(1-s)*l.first[i]+s*l.second[i];
	if( (p-result).normsqr()>(p-l.first).normsqr() )  s=0;
    if( (p-result).normsqr()>(p-l.second).normsqr() ) s=1;

    return s;
}

inline void Mouth2PA
(Line& PA, list<se_mouth*>& mouth, MouthLink & mouth_link)
{
	typedef list<se_mouth*>::iterator MIT;
	for( MIT im=mouth.begin();im!=mouth.end();im++ ){
	    se_mouth * m=*im;
        float u=(float)ClosestPt(PA,m->com,m->n);
        mouth_link.push_back(pair<float,Point2d>(u,m->com));
	}	
}

///////////////////////////////////////////////////////////////////////////////
//
// Principle Axis 
//
///////////////////////////////////////////////////////////////////////////////

inline pair<double,double> FindExtrem
(const vector<Point2d>& ptV, const Point2d& com, const Vector2d& v)
{
	double min_v=1e01;
	double max_v=-1e10;
	vector<Point2d>::const_iterator i;
	for( i=ptV.begin();i!=ptV.end();i++ ){
		const Point2d& pt=*i;
		double dot=(pt-com)*v;
		if( dot>max_v ){ max_v=dot; }
		if( dot<min_v ){ min_v=dot; }
	}

	return pair<double,double>(min_v,max_v);
}

inline Point2d ComputeCOM(const vector<Point2d>& ptV)
{
	vector<Point2d>::const_iterator i;
	Point2d com;
	for( i=ptV.begin();i!=ptV.end();i++ ){
		const Point2d& pt=*i;
		for( int D=0;D<2;D++ ) com[D]+=pt[D];
	}
	int size=ptV.size();
	for( int D=0;D<2;D++ ) com[D]/=size;
	return com;
}

inline Line ComputePA(c_polygon& poly)
{
	int total_v=0;
	typedef c_polygon::iterator PIT;
	{//compute total number of v
		for( PIT ip=poly.begin();ip!=poly.end();ip++ ){
			//ip->getSize()
			//ip->updateSize();
			total_v+=ip->getSize();
		}
	}

	vector<Point2d> pts; pts.reserve(total_v);	
	for( PIT ip=poly.begin();ip!=poly.end();ip++ ){
		const c_ply& p=*ip;
		ply_vertex* ptr=p.getHead();
		do{
			const Point2d& pt=ptr->getPos();
			pts.push_back(pt);
			ptr=ptr->getNext();
		}while( ptr!=p.getHead() );
	}

	//compute com and the point list
	Point2d com=ComputeCOM(pts); //center of mass
	Vector2d v=PC(pts,com);
	Vector2d n(-v[1],v[0]);
	pair<double,double> PAE=FindExtrem(pts,com,v);

	//Second PA
	pair<double,double> SAE=FindExtrem(pts,com,n);
	Point2d com_p1=com+n*SAE.first;
	Point2d com_p2=com+n*SAE.second;
	com.set((com_p1[0]+com_p2[0])/2,(com_p1[1]+com_p2[1])/2);
	double width=(SAE.second-SAE.first);

	//Scale PA (Shrink...)
	PAE.first=PAE.first+width;
	if( PAE.first>0 ) PAE.first=0;
	PAE.second=PAE.second-width;
	if( PAE.second<0 ) PAE.second=0;

	return Line(com+v*PAE.first,com+v*PAE.second);
}

///////////////////////////////////////////////////////////////////////////////
inline SKG ExtracSkeleton_PA
(c_polygon& poly, list<se_mouth*>& mouth)
{
	SKG bone;
	//if( mouth.size()!=2 ){
		//compute principle axis
		Line PA=ComputePA(poly);

		//connect each mouth to the pa
		MouthLink mouth_link;
		Mouth2PA(PA, mouth, mouth_link);

		//simplify the link
		vector<float> ma_pt;
		SimplifyLink(PA,mouth_link,ma_pt);

		//Build SKG
		CreateSKG(bone,mouth_link,ma_pt,PA);
	//}
	/*
	else{
		sk_node n1,n2;
		n1.m_pos=mouth.front()->com;
		n2.m_pos=mouth.back()->com;
		VID id1=bone.AddVertex(n1);
		VID id2=bone.AddVertex(n2);

		//make connection
		bone.AddEdge(id1,id2,sk_edge(1));
	}
	*/

	return bone;
}

void ES_PA::extract(se_m * m)
{
	m->skeleton=ExtracSkeleton_PA((*(m->cd)),m->mouth);
}

///////////////////////////////////////////////////////////////////////////////
inline Point2d ComputeCOM(c_polygon& poly)
{
	typedef c_polygon::iterator PIT;
	vector<Point2d> pts;
	for( PIT ip=poly.begin();ip!=poly.end();ip++ ){
		const c_ply& p=*ip;
		ply_vertex* ptr=p.getHead();
		do{
			const Point2d& pt=ptr->getPos();
			pts.push_back(pt);
			ptr=ptr->getNext();
		}while( ptr!=p.getHead() );
	}

	//compute com and the point list
	return ComputeCOM(pts); //center of mass
}

void ES_Centers::extract(se_m * m)
{
	SKG bone;
	//m->skeleton=ExtracSkeleton_PA(m->cd,m->mouth);
	list<se_mouth*>& mouth=m->mouth;
	int msize=mouth.size();
	if( msize==2 ){
		sk_node n1,n2;
		n1.m_pos=mouth.front()->com;
		n2.m_pos=mouth.back()->com;
		VID id1=bone.AddVertex(n1);
		VID id2=bone.AddVertex(n2);

		//make connection
		bone.AddEdge(id1,id2,sk_edge(1));
	}else{ //msize==1 or >2
		//compute center of mass
		sk_node com_node;
		com_node.m_pos=ComputeCOM(*(m->cd));
		VID com_id=bone.AddVertex(com_node);

		//for each mouth's com connect to this com
		for(list<se_mouth*>::iterator i=mouth.begin();i!=mouth.end();i++){
			sk_node n;
			n.m_pos=(*i)->com;
			VID id=bone.AddVertex(n);
			//make connection
			bone.AddEdge(id,com_id,sk_edge(1));
		}//end for
	}
	m->skeleton=bone;
}
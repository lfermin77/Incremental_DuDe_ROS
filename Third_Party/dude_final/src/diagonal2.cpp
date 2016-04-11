#include <fstream>
using namespace std;

#include "diagonal2.h"
#include "bpc.h"
#include "polygon.h"
#include "intersection.h"
#include "dude_util.h"

c_diagonal::c_diagonal() 
{
	c_diagonal(NULL,NULL);
}

c_diagonal::c_diagonal(const c_diagonal& other)
{
	this->m_v[0] = other.m_v[0];
	this->m_v[1] = other.m_v[1];
	this->m_len = other.m_len;
	this->quality = other.quality;
}

c_diagonal::c_diagonal(ply_vertex * s, ply_vertex * t)
{
	//inHole = false;
    m_v[0]=s;
    m_v[1]=t;
    m_len=0;
    if(s!=NULL&&t!=NULL) m_len=(s->getPos()-t->getPos()).norm();
    this->quality = 0;
}
void c_diagonal::setEndPoints(ply_vertex*s,ply_vertex*t)
{
	m_v[0] = s;
	m_v[1] = t;
	m_len = 0;
	if(s!=NULL&&t!=NULL) m_len = (s->getPos()-t->getPos()).norm();
}

//does this current diagonal cut the
//dihedral angle of v to <PI?
bool c_diagonal::is_angle_resolving(ply_vertex * v, double tau) const
{
    if(v->getExtra().isPM()==false) return true; //any diagonal will resolve non pocket minimum

    ply_vertex * other=getOther(v);

    const Point2d& v1=v->getExtra().getDihedralPre()->getPos();
    double angle1=angleAt(v1,v->getPos(),other->getPos());

    if(angle1>tau) return false;

    const Point2d& v2=v->getExtra().getDihedralNext()->getPos();
    double angle2=angleAt(other->getPos(),v->getPos(),v2);

    if(angle2>tau) return false;

    return true;
}

//check if this diagonal can resolve vertex v by determining the diagonal angles and residual concavity
bool c_diagonal::is_resolving(ply_vertex * v, double tau_concavity, double tau_angle) const
{
    if(v->getExtra().isPM()==false) return true; //any diagonal will resolve non pocket minimum

    const Point2d& other=getOther(v)->getPos();

    const Point2d& v0=v->getPos();
    const Point2d& v1=v->getExtra().getDihedralPre()->getPos();
    const Point2d& v2=v->getExtra().getDihedralNext()->getPos();

    double angle1=angleAt(v1,v0,other);
    double angle2=angleAt(other,v0,v2);

    if(angle1<tau_angle && angle2<tau_angle) return true;

    if(angle1>=tau_angle){
        double rc=dist2Seg(v1,other,v0);
        if(rc>tau_concavity) return false; //residual concavity is too large
    }

    if(angle2>=tau_angle){
        double rc=dist2Seg(v2,other,v0);
        if(rc>tau_concavity) return false; //residual concavity is too large
    }

    //residual concavity is acceptable
    return true;
}

//check if this diagonal, together with another diagonal, od,
//can resolve vertex v by determining the diagonal
//angles and residual concavity tau is concavity tolerance
bool c_diagonal::is_resolving(const c_diagonal& od, ply_vertex * v, double tau_concavity, double tau_angle) const
{
    if(v->getExtra().isPM()==false) return true; //any diagonal will resolve non pocket minimum

    const Point2d& o1=getOther(v)->getPos();
    const Point2d& o2=od.getOther(v)->getPos();

    const Point2d& v0=v->getPos();
    const Point2d& v1=v->getExtra().getDihedralPre()->getPos();
    const Point2d& v2=v->getExtra().getDihedralNext()->getPos();

    double angle1=angleAt(v1,v0,o1);
    double angle2=angleAt(o1,v0,o2);
    double angle3=angleAt(o2,v0,v2);

    if(angle1<tau_angle && angle2<tau_angle && angle3<tau_angle) return true;

    if(angle1>=tau_angle){
        double rc=dist2Seg(v1,o1,v0);
        if(rc>tau_concavity) return false; //residual concavity is too large
    }

    if(angle2>=tau_angle){
        double rc=dist2Seg(o1,o2,v0);
        if(rc>tau_concavity) return false; //residual concavity is too large
    }

    if(angle3>=tau_angle){
        double rc=dist2Seg(o2,v2,v0);
        if(rc>tau_concavity) return false; //residual concavity is too large
    }

    //residual concavity is acceptable
    return true;
}

double c_diagonal::angleAt(const Point2d& p1, const Point2d& p2, const Point2d& p3) const
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

bool c_diagonal::operator==(const c_diagonal& other) const
{
    if(m_v[0]==other.m_v[0] && m_v[1]==other.m_v[1]) return true;
    if(m_v[0]==other.m_v[1] && m_v[1]==other.m_v[0]) return true;
    return false;
}


void saveDiagonals(vector<c_diagonal>& diags, string filename)
{
	typedef vector<c_diagonal>::iterator DIT;

	cerr<<"diagonals saved to "<<filename<<"\n";

	ofstream fout;
	fout.open(filename.c_str());

	fout<<"1\n"<<diags.size()<<" 3\n";
	for(DIT it=diags.begin(); it!=diags.end();++it)
		fout<<it->getV1()->getPos()<<it->getV2()->getPos()<<"\n";

	fout.close();
}

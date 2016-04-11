#ifndef __DIAGONAL2_H__
#define __DIAGONAL2_H__

#include "Point.h"
using namespace mathtool;

#include <cassert>
#include <vector>
using namespace std;

class ply_vertex; //defined in polygon.h
class c_plygon;

class c_diagonal
{

public:

    c_diagonal();
    c_diagonal(const c_diagonal& other);
    c_diagonal(ply_vertex * s, ply_vertex * t);

    ply_vertex * getV1() const {return m_v[0]; }
    ply_vertex * getV2() const {return m_v[1]; }
    ply_vertex * getOther(ply_vertex * v) const {
        if(m_v[0]==v) return m_v[1];
        else if (m_v[1]==v) return m_v[0];
        else assert(false);
        return NULL;
    }
	void setEndPoints(ply_vertex*s,ply_vertex*t);

    bool operator==(const c_diagonal& other) const;

    //check if this diagonal can resolve vertex v by determining the diagonal angles
    //tau is the angle tolerance
    bool is_angle_resolving(ply_vertex * v, double tau=PI) const;

    //check if this diagonal can resolve vertex v by determining the diagonal angles and residual concavity
    //tau is concavity tolerance
    bool is_resolving(ply_vertex * v, double tau_concavity, double tau_angle=PI) const;

    //check if this diagonal, together with another diagonal, od,
    //can resolve vertex v by determining the diagonal
    //angles and residual concavity tau is concavity tolerance
    bool is_resolving(const c_diagonal& od, ply_vertex * v, double tau_concavity, double tau_angle=PI) const;

    //lenght of this diagonal
    double getLength() const { return m_len; }

    int getQuality() const { return this->quality;}

    void setQuality(int value) { this->quality = value; }

protected:

    //bool is_bridge_breaking(ply_vertex * v, ply_vertex * u);
    double angleAt(const Point2d& p1, const Point2d& p2, const Point2d& p3) const;

private:

    ply_vertex * m_v[2];
    double m_len;
    int quality;		// added by Zhonghua 4/7/2013
};


void saveDiagonals(vector<c_diagonal>& diags, string filename);

#endif


#pragma once

#include "diagonal2.h"

#include "Point.h"
using namespace mathtool;

#include <vector>
#include <list>
using namespace std;

class ply_vertex; //defined in polygon.h

class c_cutset
{
public:

    enum Type {SINGLE_PM, SINGLE_TIP, DOUBLE_PM_PM, DOUBLE_PM_TIP, DOUBLE_TIP_TIP, UNKNOW  };

    //cutset constructor
    c_cutset(){ m_resolves_n_PM=0; m_v=NULL; m_weight=-1; m_type=UNKNOW; }


    //a static method for building all cutsets for a given vertex v
    static void build_cutsets(ply_vertex * v, const Point2d& start, const Point2d& end,
                              const vector<c_diagonal>& diagonals, double tau,
                              vector<c_cutset>& cutset_list);


    //get diagonals
    const vector<c_diagonal>& getDiagonals() const { return m_diagonals; }

    //return the complement of this cutset
    //vector<c_cutset> complement();

    //check if the given cutset conflicts with this cutset
    bool conflict(const c_cutset& other) const;

    //check if the given cutset conflicts with this cutset at diagonal
    bool conflict(const c_cutset& other, const c_diagonal& diagonal) const;

    //comparison
    bool operator==(const c_cutset& other) const;

    //access
    short getResolveSize() const { return m_resolves_n_PM; }


    ply_vertex * getV() const { return m_v; }

    void setType(Type t) { m_type=t; }
    Type getType() const { return m_type; }

    //weight related functions

    double getWeight() const { assert(m_weight>=0); return m_weight; }

protected:

    static void handle_single_PM(list<c_cutset>& cutsets);
    static void handle_single_TIP(list<c_cutset>& cutsets);
    static void handle_double_PM_PM(list<c_cutset>& cutsets);
    static void handle_double_PM_TIP(list<c_cutset>& cutsets);
    static void handle_double_TIP_TIP(list<c_cutset>& cutsets);

    //analyze the cutset
    short analyze_single_PM();
    short analyze_single_TIP();
    short analyze_double_PM_PM();
    short analyze_double_PM_TIP();
    short analyze_double_TIP_TIP();

    //set the number of pms that this cutset can resolve
    void setResolveSize(short n);

    //
    bool isMemberOf(const vector<c_cutset>& cutsets) const;

    //determine how good is this cutset
    double computeWeight();

private:

    short m_resolves_n_PM; //this c_cutset can resolve n pocket minimum
    Type m_type;

    static double m_tau; //this is specified through build_cutsets

    double m_weight;
    ply_vertex * m_v; //this cutset can resolve this vertex m_v
    vector<c_diagonal> m_diagonals; //these diagonals together will resolve m_v
};


ostream& operator<<(ostream& out, const c_cutset& cset);


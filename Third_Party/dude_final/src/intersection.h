//------------------------------------------------------------------------------
//  Copyright 2010-2011 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _MKSUM_INTERSECTION_H_
#define _MKSUM_INTERSECTION_H_

#include "Basic.h"
//#include "mpfr.h"

// This is copied/modified from CG in C

template<class real>
void Assign( real p[2], real a[2])
{
   p[0] = a[0];
   p[1] = a[1];
}

template<class real> real Area(const real a[2], const real b[2], const real c[2])
{
    return ( b[0] - a[0] ) * ( c[1] - a[1] ) -
           ( c[0] - a[0] ) * ( b[1] - a[1] );
}

template<class real> int AreaSign(const real a[2], const real b[2], const real c[2])
{
    real area=Area(a,b,c);

    //if( sizeof(a[0])<=sizeof(double) ){
    //    if( (area>0 && area<SMALLNUMBER) || (area<0 && area>-SMALLNUMBER) )
    //    {
    //        mpreal A[2]; A[0]=a[0]; A[1]=a[1];
    //        mpreal B[2]; B[0]=b[0]; B[1]=b[1];
    //        mpreal C[2]; C[0]=c[0]; C[1]=c[1];
    //        return AreaSign<mpreal>(A,B,C);
    //    }
    //}

    if      ( area >  0 ) return  1;
    else if ( area <  0 ) return -1;
    else     return  0;
}

template<class real> int Collinear(const real a[2], const real b[2], const real c[2])
{
   return AreaSign( a, b, c ) == 0;
}

/*---------------------------------------------------------------------
Returns TRUE iff point c lies on the closed segement ab.
Assumes it is already known that abc are collinear.
---------------------------------------------------------------------*/
template<class real> bool Between(const real a[2], const real b[2], const real c[2])
{
   // If ab not vertical, check betweenness on x; else on y.
   if ( a[0]!=b[0] )
      return ((a[0] <= c[0]) && (c[0] <= b[0])) ||
             ((a[0] >= c[0]) && (c[0] >= b[0]));
   else
      return ((a[1] <= c[1]) && (c[1] <= b[1])) ||
             ((a[1] >= c[1]) && (c[1] >= b[1]));
}

template<class real> bool Between_strict(const real a[2], const real b[2], const real c[2])
{
   // If ab not vertical, check betweenness on x; else on y.
    if ( a[0]!=b[0] )
    {
      return ((a[0] < c[0]) && (c[0] < b[0])) ||
             ((a[0] > c[0]) && (c[0] > b[0]));
    }
    else{
        return ((a[1] < c[1]) && (c[1] < b[1])) ||
               ((a[1] > c[1]) && (c[1] > b[1]));
    }
}


template<class real> bool AlmostEqual3(const real a[3], const real b[3])
{
    return (fabs(a[0]-b[0])<SMALLNUMBER &&fabs(a[1]-b[1])<SMALLNUMBER && fabs(a[2]-b[2])<SMALLNUMBER);
}

template<class real> bool AlmostEqual(const real a[2], const real b[2])
{
    return (fabs(a[0]-b[0])<SMALLNUMBER &&fabs(a[1]-b[1])<SMALLNUMBER);
}

template<class real> bool Equal3(const real a[3], const real b[3])
{
    return (a[0]==b[0])&& (a[1]==b[1]) && (a[2]==b[2]);
}

template<class real> bool Equal(const real a[2], const real b[2])
{
    return (a[0]==b[0])&& (a[1]==b[1]);
}

template<class real> char ParallelInt
(const real a[2], const real b[2], const real c[2], const real d[2], real p[2])
{
    if(!Collinear(a, b, c)) return '0';

    //check if they overlap..
    if(Between(a,b,c)) return 'e';
    if(Between(a,b,d)) return 'e';
    if(Between(c,d,a)) return 'e';
    if(Between(c,d,b)) return 'e';

    //they don't overlap but the end points may..
    //check if the end points overlap
    if(AlmostEqual(a,c)){ p[0]=a[0]; p[1]=a[1]; return 'v';}
    if(AlmostEqual(b,c)){ p[0]=b[0]; p[1]=b[1]; return 'v';}
    if(AlmostEqual(a,d)){ p[0]=a[0]; p[1]=a[1]; return 'v';}
    if(AlmostEqual(b,d)){ p[0]=b[0]; p[1]=b[1]; return 'v';}

    return '0';
}


// compute union of two colinear  segments ab and cd
// place the union in p
// return false if the union is degenerated
template<class real> bool Union
(const real a[2], const real b[2], const real c[2], const real d[2],
 const real * p[2])
{
    int id=0;
    if(Equal(a,c)){ p[id]=a; id++; }
    
    if(Equal(a,d)){ p[id]=a; id++; }  
    if(id==2) return p[0]!=p[1];
    
    if(Equal(b,c)){ p[id]=b; id++; }  
    if(id==2) return p[0]!=p[1];

    if(Equal(b,d)){ p[id]=b; id++; }  
    if(id==2) return p[0]!=p[1];

    if( Between_strict(a,b,c) ){ p[id]=c; id++; } 
    if(id==2) return p[0]!=p[1];
    
    if( Between_strict(a,b,d) ){ p[id]=d; id++; } 
    if(id==2) return p[0]!=p[1];

    if( Between_strict(c,d,a) ){ p[id]=a; id++; }
    if(id==2) return p[0]!=p[1];

    if( Between_strict(c,d,b) ){ p[id]=b; id++; }
    if(id==2) return p[0]!=p[1];

    return false;
}

/*---------------------------------------------------------------------
SegSegInt: Detect intersection between two closed segments ab and cd.
---------------------------------------------------------------------*/


template<class real> bool SegSegInt( const real a[2], const real b[2],
                       const real c[2], const real d[2])
{
    //check X and Y coordinates
    for(int i=0;i<2;i++){
        if(a[i]<b[i]){
            if(c[i]<d[i]){
                if( a[i]>d[i] ) return false;
                if( c[i]>b[i] ) return false;
            }
            else{ //c[i]>=d[i]
                if( a[i]>c[i] ) return false;
                if( d[i]>b[i] ) return false;
            }
        }
        else{ //a[i]>=b[i]
            if(c[i]<d[i]){
                if( b[i]>d[i] ) return false;
                if( c[i]>a[i] ) return false;
            }
            else{ //c[i]>=d[i]
                if( b[i]>c[i] ) return false;
                if( d[i]>a[i] ) return false;
            }
        }
    }//end for i

    //OK potential intersection
    int abc=AreaSign(a,b,c);
    int abd=AreaSign(a,b,d);

    if(abc==0 && abd==0 ) { //collinear
        //check if they overlap..
        if(Between(a,b,c)) return true;
        if(Between(a,b,d)) return true;
        if(Between(c,d,a)) return true;
        if(Between(c,d,b)) return true;
        return false;
    }
    else if(abc==0){
        if(Between(a,b,c)) return true;
        return false;
    }
    else if(abd==0){
        if(Between(a,b,d)) return true;
        return false;
    }
    //
    else{ // if(abc!=0 && abd!=0)
        if(abc>0 && abd>0) return false;
        if(abc<0 && abd<0) return false;
    }

    int cda=AreaSign(c,d,a);
    int cdb=AreaSign(c,d,b);

    assert(cda!=0 || cdb!=0);

    if(cda==0){
        if(Between(c,d,a)) return true;
        return false;
    }
    else if(cdb==0){
        if(Between(c,d,b)) return true;
        return false;
    }
    else{
        if(cda>0 && cdb>0) return false;
        if(cda<0 && cdb<0) return false;
    }

    return true;
}

/*---------------------------------------------------------------------
SegSegInt: Finds the point of intersection p between two closed
segments ab and cd.  Returns p and a char with the following meaning:
   'e': The segments collinearly overlap, sharing a point.
   'v': An endpoint (vertex) of one segment is on the other segment,
        but 'e' doesn't hold.
   '1': The segments intersect properly (i.e., they share a point and
        neither 'v' nor 'e' holds).
   '0': The segments do not intersect (i.e., they share no points).
Note that two collinear segments that share just one point, an endpoint
of each, returns 'e' rather than 'v' as one might expect.
---------------------------------------------------------------------*/
template<class real>
char SegSegInt( const real a[2], const real b[2],
                const real c[2], const real d[2],
                real p[2] )
{
    real  s, t;                  // The two parameters of the parametric eqns.
    real  num_s, num_t, denom;   // Numerator and denoninator of equations.
    char  code = '?';            // Return char characterizing intersection.


    //
    if(a[0]==c[0] && a[1]==c[1]){ p[0]=a[0]; p[1]=a[1]; return 'v'; }
    if(b[0]==c[0] && b[1]==c[1]){ p[0]=b[0]; p[1]=b[1]; return 'v'; }
    if(a[0]==d[0] && a[1]==d[1]){ p[0]=a[0]; p[1]=a[1]; return 'v'; }
    if(b[0]==d[0] && b[1]==d[1]){ p[0]=b[0]; p[1]=b[1]; return 'v'; }
    //

    denom = a[0] * ( d[1] - c[1] ) +
            b[0] * ( c[1] - d[1] ) +
            d[0] * ( b[1] - a[1] ) +
            c[0] * ( a[1] - b[1] );

// comment out by Zhonghua, currently we don't need high precision
// 1/26/2015
// #if defined(__APPLE__) || defined(__linux__)


//     if(fabs(denom)<SMALLNUMBER)
//     {
//         //ok, we are using double, may need more precision
//         if( sizeof(a[0])<=sizeof(double) ){
//             mpreal A[2]; A[0]=a[0]; A[1]=a[1];
//             mpreal B[2]; B[0]=b[0]; B[1]=b[1];
//             mpreal C[2]; C[0]=c[0]; C[1]=c[1];
//             mpreal D[2]; D[0]=d[0]; D[1]=d[1];
//             mpreal P[2];
//             char r=SegSegInt<mpreal>(A,B,C,D,P);
//             p[0]=P[0];
//             p[1]=P[1];
//             return r;
//         }
//     }
// #endif

    // If denom is zero, then segments are parallel: handle separately.
    if (denom==0) return  ParallelInt(a, b, c, d, p);

    //compute s
    num_s =    a[0] * ( d[1] - c[1] ) +
               c[0] * ( a[1] - d[1] ) +
               d[0] * ( c[1] - a[1] );

    if( fabs(num_s)<SMALLNUMBER ) num_s=0;
    else if( fabs(num_s-denom)<SMALLNUMBER ) num_s=denom;

    s = num_s / denom;

    if(s<0  || s>1) return '0';

    //compute t
    num_t = -( a[0] * ( c[1] - b[1] ) +
               b[0] * ( a[1] - c[1] ) +
               c[0] * ( b[1] - a[1] ) );

    if( fabs(num_t)<SMALLNUMBER ) num_t=0;
    else if( fabs(num_t-denom)<SMALLNUMBER) num_t=denom;

    t = num_t / denom;
    if(t<0  || t>1) return '0';

    //decide the code
    if( (0.0<s) && (s< 1.0) && (0.0< t) && (t< 1.0) )
         code = '1';
    //else if ( (0.0>s) || (s>1.0) || (0.0>t) || (t>1.0) )
    //     return '0';
    else {
        code= 'v';
    }

    p[0] = (a[0] + s*(b[0]-a[0]));
    p[1] = (a[1] + s*(b[1]-a[1]));

    /*
    if(code=='v'){
        real q[2];
        q[0] = (c[0] + t*(d[0]-c[0]));
        q[1] = (c[1] + t*(d[1]-c[1]));
        if(AlmostEqual(p,q)==false){
            return '0';
        }
    }
    */

    return code;
}



////////////////////////////////////////////////////////////////////////////////
// This is from RAPID and should be simplified


template<class real> real
VdotV(real V1[3], real V2[3])
{
  return (V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]);
}



template<class real> void
VcrossV(real Vr[3], const real V1[3], const real V2[3])
{
  Vr[0] = V1[1]*V2[2] - V1[2]*V2[1];
  Vr[1] = V1[2]*V2[0] - V1[0]*V2[2];
  Vr[2] = V1[0]*V2[1] - V1[1]*V2[0];
}

template<class real> real
max(real a, real b, real c)
{
  real t = a;
  if (b > t) t = b;
  if (c > t) t = c;
  return t;
}
 
template<class real> real
min(real a, real b, real c)
{
  real t = a;
  if (b < t) t = b;
  if (c < t) t = c;
  return t;
}


template<class real> int
my_project6_2(real *ax,
     real *p1, real *p2, real *p3,
     real *q1, real *q2, real *q3)
{
  real P1 = VdotV(ax, p1);
  real P2 = VdotV(ax, p2);
  real P3 = VdotV(ax, p3);
  real Q1 = VdotV(ax, q1);
  real Q2 = VdotV(ax, q2);
  real Q3 = VdotV(ax, q3);
  
  real mx1 = max(P1, P2, P3);
  real mn1 = min(P1, P2, P3);
  real mx2 = max(Q1, Q2, Q3);
  real mn2 = min(Q1, Q2, Q3);

  if (mn1 > mx2) return 0;
  if (mn2 > mx1) return 0;
  return 1;
}


// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles

template<class real> int
my_tri_contact (real *P1, real *P2, real *P3,
                real *Q1, real *Q2, real *Q3)
{

  /*
     One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
     Edges are (e1,e2,e3) and (f1,f2,f3).
     Normals are n1 and m1
     Outwards are (g1,g2,g3) and (h1,h2,h3).

     We assume that the triangle vertices are in the same coordinate system.

     First thing we do is establish a new c.s. so that p1 is at (0,0,0).
     */

  real p1[3], p2[3], p3[3];
  real q1[3], q2[3], q3[3];
  real e1[3], e2[3], e3[3];
  real f1[3], f2[3], f3[3];
  real g1[3], g2[3], g3[3];
  real h1[3], h2[3], h3[3];
  real n1[3], m1[3];
  real z[3];

  real ef11[3], ef12[3], ef13[3];
  real ef21[3], ef22[3], ef23[3];
  real ef31[3], ef32[3], ef33[3];
  
  z[0] = 0.0;  z[1] = 0.0;  z[2] = 0.0;
  
  p1[0] = P1[0] - P1[0];  p1[1] = P1[1] - P1[1];  p1[2] = P1[2] - P1[2];
  p2[0] = P2[0] - P1[0];  p2[1] = P2[1] - P1[1];  p2[2] = P2[2] - P1[2];
  p3[0] = P3[0] - P1[0];  p3[1] = P3[1] - P1[1];  p3[2] = P3[2] - P1[2];
  
  q1[0] = Q1[0] - P1[0];  q1[1] = Q1[1] - P1[1];  q1[2] = Q1[2] - P1[2];
  q2[0] = Q2[0] - P1[0];  q2[1] = Q2[1] - P1[1];  q2[2] = Q2[2] - P1[2];
  q3[0] = Q3[0] - P1[0];  q3[1] = Q3[1] - P1[1];  q3[2] = Q3[2] - P1[2];
  
  e1[0] = p2[0] - p1[0];  e1[1] = p2[1] - p1[1];  e1[2] = p2[2] - p1[2];
  e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];

  f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];
  
  VcrossV(n1, e1, e2);
  VcrossV(m1, f1, f2);

  VcrossV(g1, e1, n1);
  VcrossV(g2, e2, n1);
  VcrossV(g3, e3, n1);
  VcrossV(h1, f1, m1);
  VcrossV(h2, f2, m1);
  VcrossV(h3, f3, m1);

  VcrossV(ef11, e1, f1);
  VcrossV(ef12, e1, f2);
  VcrossV(ef13, e1, f3);
  VcrossV(ef21, e2, f1);
  VcrossV(ef22, e2, f2);
  VcrossV(ef23, e2, f3);
  VcrossV(ef31, e3, f1);
  VcrossV(ef32, e3, f2);
  VcrossV(ef33, e3, f3);
  
  // now begin the series of tests

  if (!my_project6_2(n1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(m1, p1, p2, p3, q1, q2, q3)) return 0;
  
  if (!my_project6_2(ef11, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef12, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef13, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef21, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef22, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef23, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef31, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef32, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(ef33, p1, p2, p3, q1, q2, q3)) return 0;

  if (!my_project6_2(g1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(g2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(g3, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(h1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(h2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!my_project6_2(h3, p1, p2, p3, q1, q2, q3)) return 0;

  return 1;
}

template<class real> real
distance2D(real a[2], real b[2])
{
	return sqrtf((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]));
}

template<class real> real
distance2D(const real& a1, const real& b1, const real& a2, const real& b2)
{
	return sqrtf((a1 - a2)*(a1 - a2) + (b1 - b2)*(b1 - b2));
}

#endif//_MKSUM_INTERSECTION_H_



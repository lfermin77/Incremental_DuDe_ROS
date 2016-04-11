//------------------------------------------------------------------------------
//  Copyright 2007-2014 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------


#ifndef _H_Matrix_H_
#define _H_Matrix_H_

#include "Vector.h"

namespace mathtool{

    //
    // The internal storage form of transforms.  The matrices in row-major
    // order (ie  mat[row][column] )
    //

    class Matrix2x2 {

    protected:

        Vector2d row[2];

    public:

        Matrix2x2(double a11 = 1, double a12 = 0,
            double a21 = 0, double a22 = 1)
        {
            set(a11, a12, a21, a22);
        }

        //
        // return row i of matrix as a vector
        // Routines returning an lvalue: i.e. M[i] returns ref to row i
        //
        Vector2d& operator[](int i)
        {
            if(i < 0 || i > 1){
                cerr << "Matrix2x2 row index " << i << " out of bounds" << endl;
                exit(1);
            }
            return (row[i]);
        }


        const Vector2d& operator[](int i) const
        {
            if(i < 0 || i > 1){
                cerr << "Matrix2x2 row index " << i << " out of bounds" << endl;
                exit(1);
            }
            return (row[i]);
        }

        void print(int w = 7, int p = 3) const // print with width and precision
        {
            for(int i = 0; i < 2; i++){
                cout << setw(w) << setprecision(p) << round(row[i][0], p) << " ";
                cout << setw(w) << setprecision(p) << round(row[i][1], p);
                cout << endl;
            }
        }

        void set(double a11 = 0, double a12 = 0, double a21 = 0, double a22 = 0)
        {
            row[0].set(a11, a12);
            row[1].set(a21, a22);
        }

        template<class T>
        void set(T value[2][2]){
            for(int i=0;i<2;i++)
                for(int j=0;j<2;j++)
                    row[i][j]=value[i][j];
        }


        void get(double value[2][2]) const
        {
            for(int i=0;i<2;i++)
                for(int j=0;j<2;j++)
                    value[i][j]=row[i][j];
        }

        void identity()
        {
            set(1, 0, 0, 1);
        }

        Matrix2x2 transpose() const 
        {
            Matrix2x2 transM;

            transM[0][0] = row[0][0];
            transM[1][0] = row[0][1];
            transM[0][1] = row[1][0];
            transM[1][1] = row[1][1];

            return transM;
        }

        Matrix2x2 inv() const
        {
            Matrix2x2 invM;
            double d;

            d = row[0][0]*row[1][1] - row[0][1]*row[1][0];

            if(d == 0.0)
                cerr << "inverse of singular Matrix2x2" << endl;

            invM[0][0] = row[1][1] / d;
            invM[0][1] = -row[0][1] / d;
            invM[1][0] = -row[1][0] / d;
            invM[1][1] = row[0][0] / d;

            return invM;
        }

        double trace() const
        {
            return row[0][0]+row[1][1];
        }

        Matrix2x2 operator*(double a) const
        {
            Matrix2x2 result;

            for(int i = 0; i < 2; i++){
                result.row[i][0] = a * row[i][0];
                result.row[i][1] = a * row[i][1];
            }

            return result;
        }

        friend Matrix2x2 operator+(const Matrix2x2& m1, const Matrix2x2& m2);
        friend Matrix2x2 operator-(const Matrix2x2& m1, const Matrix2x2& m2);
        friend Matrix2x2 operator*(const Matrix2x2& m1, const Matrix2x2& m2);
        friend Matrix2x2 operator*(double a, const Matrix2x2& m);


        // mat times vector
        friend Vector2d operator*(const Matrix2x2& m, const Vector2d& v);

        // vector times mat
        friend Vector2d operator*(const Vector2d& v, const Matrix2x2& m);

        // outer product
        friend Matrix2x2 operator&(const Vector2d& v1, const Vector2d& v2);
    };

    class Matrix3x3 {

    protected:

        Vector3d row[3];

    public:

        Matrix3x3(double a11=1, double a12=0, double a13=0,
            double a21=0, double a22=1, double a23=0,
            double a31=0, double a32=0, double a33=1)
        {
            set(a11, a12, a13, a21, a22, a23, a31, a32, a33);
        }

        Vector3d& operator[](int i){
            if(i < 0 || i > 2){
                cerr << "Matrix3x3 row index " << i << " out of bounds" << endl;
                exit(1);
            }

            return (row[i]);
        }

        const Vector3d& operator[](int i) const{
            if(i < 0 || i > 2){
                cerr << "Matrix3x3 row index " << i << " out of bounds" << endl;
                exit(1);
            }

            return (row[i]);
        }

        void print(int w = 7, int p = 3) const{  // print with width and precision

            for(int i = 0; i < 3; i++){
                cout << setw(w) << setprecision(p) << round(row[i][0], p) << " ";
                cout << setw(w) << setprecision(p) << round(row[i][1], p) << " ";
                cout << setw(w) << setprecision(p) << round(row[i][2], p);
                cout << endl;
            }
        }

        void set(double a11, double a12, double a13,
            double a21, double a22, double a23,
            double a31, double a32, double a33)
        {
            row[0].set(a11, a12, a13);
            row[1].set(a21, a22, a23);
            row[2].set(a31, a32, a33);
        }

        template<class T>
        void set(T value[3][3]){
            for(int i=0;i<3;i++)
                for(int j=0;j<3;j++)
                    row[i][j]=value[i][j];
        }

        template<class T>
        void get(T value[3][3]) const
        {
            for(int i=0;i<3;i++)
                for(int j=0;j<3;j++)
                    value[i][j]=row[i][j];
        }

        void identity(){
            set(1, 0, 0, 0, 1, 0, 0, 0, 1);
        }

        Matrix3x3 transpose() const {
            Matrix3x3 transM;

            transM[0][0] = row[0][0];
            transM[1][0] = row[0][1];
            transM[2][0] = row[0][2];
            transM[0][1] = row[1][0];
            transM[1][1] = row[1][1];
            transM[2][1] = row[1][2];
            transM[0][2] = row[2][0];
            transM[1][2] = row[2][1];
            transM[2][2] = row[2][2];

            return transM;
        }

        Matrix3x3 inv() const {
            Matrix3x3 invM;
            double d;

            d = row[0][0]*row[1][1]*row[2][2] + row[0][1]*row[1][2]*row[2][0] +
                row[0][2]*row[2][1]*row[1][0] - row[0][2]*row[1][1]*row[2][0] - 
                row[0][1]*row[1][0]*row[2][2] - row[0][0]*row[2][1]*row[1][2];

            if(d == 0.0)
                cerr << "inverse of singular Matrix3x3" << endl;

            invM[0][0] = (row[1][1]*row[2][2] - row[1][2]*row[2][1]) / d;
            invM[0][1] = (row[0][2]*row[2][1] - row[0][1]*row[2][2]) / d;
            invM[0][2] = (row[0][1]*row[1][2] - row[0][2]*row[1][1]) / d;
            invM[1][0] = (row[1][2]*row[2][0] - row[1][0]*row[2][2]) / d;
            invM[1][1] = (row[0][0]*row[2][2] - row[0][2]*row[2][0]) / d;
            invM[1][2] = (row[0][2]*row[1][0] - row[0][0]*row[1][2]) / d;
            invM[2][0] = (row[1][0]*row[2][1] - row[1][1]*row[2][0]) / d;
            invM[2][1] = (row[0][1]*row[2][0] - row[0][0]*row[2][1]) / d;
            invM[2][2] = (row[0][0]*row[1][1] - row[0][1]*row[1][0]) / d;

            return invM;
        }

        double trace() const {
            return row[0][0]+row[1][1]+row[2][2];
        }

        Matrix3x3 operator*(double a) const
        {
            Matrix3x3 result;
            int i, j;

            for(i = 0; i < 3; i++)
                for(j = 0; j < 3; j++)
                    result[i][j] = a * row[i][j];

            return result;
        }

        friend Matrix3x3 operator+(const Matrix3x3& m1, const Matrix3x3& m2);
        friend Matrix3x3 operator-(const Matrix3x3& m1, const Matrix3x3& m2);
        friend Matrix3x3 operator*(const Matrix3x3& m1, const Matrix3x3& m2);
        friend Matrix3x3 operator*(double a, const Matrix3x3& m);


        // mat times vector
        friend Vector3d operator*(const Matrix3x3& m, const Vector3d& v);
        // vector times mat
        friend Vector3d operator*(const Vector3d& v, const Matrix3x3& m);
        // outer product
        friend Matrix3x3 operator&(const Vector3d& v1, const Vector3d& v2);

        // return a rotation Matrix around X axis for theta radius
        static Matrix3x3 rotX(const double theta)
        {
            Matrix3x3 m(1, 0, 0,
                       0, cos(theta), -sin(theta),
                       0, sin(theta), cos(theta)
                       );
            return m;
        }

        // return a rotation Matrix around Y axis for theta radius
        static Matrix3x3 rotY(const double theta)
        {
            Matrix3x3 m(
                       cos(theta), 0, sin(theta),
                       0,          1, 0,
                       -sin(theta), 0, cos(theta)
                       );
            return m;
        }

        // return a rotation Matrix around Y axis for theta radius
       static Matrix3x3 rotZ(const double theta)
       {
           Matrix3x3 m(
                      cos(theta), -sin(theta), 0,
                      sin(theta), cos(theta),  0,
                      0,          0,           1
                      );
           return m;
       }
    };

    class Matrix4x4 {
    protected:
        Vector4d row[4];

    public:

        Matrix4x4(double a11=1, double a12=0, double a13=0, double a14=0,
            double a21=0, double a22=1, double a23=0, double a24=0,
            double a31=0, double a32=0, double a33=1, double a34=0,
            double a41=0, double a42=0, double a43=0, double a44=1)
        {
            set(a11, a12, a13, a14, a21, a22, a23, a24,
                a31, a32, a33, a34, a41, a42, a43, a44);
        }

        //operator Matrix();

        Vector4d& operator[](int i){
            if(i < 0 || i > 3){
                cerr << "Matrix4x4 row index " << i << " out of bounds" << endl;
                exit(1);
            }

            return (row[i]);
        }

        const Vector4d& operator[](int i) const
        {
            if(i < 0 || i > 3){
                cerr << "Matrix4x4 row index " << i << " out of bounds" << endl;
                exit(1);
            }

            return (row[i]);
        }

        void print(int w = 7, int p = 3) const {  // print with width and precision

            for(int i = 0; i < 4; i++){
                cout << setw(w) << setprecision(p) << round(row[i][0], p) << " ";
                cout << setw(w) << setprecision(p) << round(row[i][1], p) << " ";
                cout << setw(w) << setprecision(p) << round(row[i][2], p) << " ";
                cout << setw(w) << setprecision(p) << round(row[i][3], p);
                cout << endl;
            }
        }

        void set(double a11, double a12, double a13, double a14,
            double a21, double a22, double a23, double a24,
            double a31, double a32, double a33, double a34,
            double a41, double a42, double a43, double a44)
        {
            row[0].set(a11, a12, a13, a14);
            row[1].set(a21, a22, a23, a24);
            row[2].set(a31, a32, a33, a34);
            row[3].set(a41, a42, a43, a44);
        }

        template<class T>
        void set(T value[4][4]){
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    row[i][j]=value[i][j];
        }

        void get(double value[4][4]) const
        {
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    value[i][j]=row[i][j];
        }

        void identity()
        {
            set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
        }

        Matrix4x4 transpose() const
        {
            Matrix4x4 transM;

            transM[0][0] = row[0][0];
            transM[1][0] = row[0][1];
            transM[2][0] = row[0][2];
            transM[3][0] = row[0][3];
            transM[0][1] = row[1][0];
            transM[1][1] = row[1][1];
            transM[2][1] = row[1][2];
            transM[3][1] = row[1][3];
            transM[0][2] = row[2][0];
            transM[1][2] = row[2][1];
            transM[2][2] = row[2][2];
            transM[3][2] = row[2][3];
            transM[0][3] = row[3][0];
            transM[1][3] = row[3][1];
            transM[2][3] = row[3][2];
            transM[3][3] = row[3][3];

            return transM;
        }

        Matrix4x4 inv() const
        {
            Matrix4x4 LU_M, invM;
            int i, j, indx[4];
            double col[4];

            LU_M = LU_Decompose(*this, indx);

            for(j = 0; j < 4; j++){
                for(i = 0; i < 4; i++)
                    col[i] = 0.0;
                col[j] = 1.0;
                LU_back_substitution(LU_M, indx, col);
                for(i = 0; i < 4; i++)
                    invM[i][j] = col[i];
            }

            return invM;
        }

        double trace() const
        {
            return row[0][0]+row[1][1]+row[2][2]+row[3][3];
        }

        Matrix4x4 operator*(double a) const
        {

            Matrix4x4 result;

            int i, j;
            for(i = 0; i < 4; i++)
                for(j = 0; j < 4; j++)
                    result[i][j] = a * row[i][j];

            return result;
        }

        friend Matrix4x4 operator+(const Matrix4x4& m1, const Matrix4x4& m2);
        friend Matrix4x4 operator-(const Matrix4x4& m1, const Matrix4x4& m2);
        friend Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2);
        friend Matrix4x4 operator*(double a, const Matrix4x4& m);


        // mat times vector
        friend Vector4d operator*(const Matrix4x4& m, const Vector4d& v);
        // vector times mat
        friend Vector4d operator*(const Vector4d& v, const Matrix4x4& m);
        // outer product
        friend Matrix4x4 operator&(const Vector4d& v1, const Vector4d& v2);

    protected:

        //////////////////////////////////////////////////////////////////////////
        // the following matrix operations are used to find the inverse of an 
        // NxN matrix. Adapted from Numerical Recipes by (Frank) Sebastian Grassia
        //////////////////////////////////////////////////////////////////////////


        Matrix4x4 LU_Decompose(const Matrix4x4& M, int *indx) const
        {
            int  i, imax, j, k;
            double big, dum, sum, temp;
            double vv[4];
            Matrix4x4 LU_M = M;
            int N = 4;

            for(i = 0; i < N; i++){
                big = 0.0;
                for(j = 0; j < N; j++)
                    if((temp = fabs(M.row[i][j])) > big)
                        big = temp;
                if(big == 0.0)
                    cerr << "inverse of singular Matrix4x4" << endl;

                vv[i] = 1.0f / big;
            }

            for(j = 0; j < N; j++){
                for(i = 0; i < j; i++){
                    sum = LU_M[i][j];
                    for(k = 0; k < i; k++)
                        sum -= LU_M[i][k] * LU_M[k][j];
                    LU_M[i][j] = sum;
                }
                big = 0.0;
                for(i = j; i < N; i++){
                    sum = LU_M[i][j];
                    for(k = 0; k < j; k++)
                        sum -= LU_M[i][k] * LU_M[k][j];
                    LU_M[i][j] = sum;
                    if((dum = vv[i]*fabs(sum)) >= big){
                        big = dum;
                        imax = i;
                    }
                }
                if(j != imax){
                    for(k = 0; k < N; k++){
                        dum = LU_M[imax][k];
                        LU_M[imax][k] = LU_M[j][k];
                        LU_M[j][k] = dum;
                    }
                    vv[imax] = vv[j];
                }
                indx[j] = imax;
                if(j < N - 1){
                    dum = 1.0f / LU_M[j][j];
                    for(i = j + 1; i < N; i++)
                        LU_M[i][j] *= dum;
                }
            }

            return LU_M;
        }

        void LU_back_substitution(const Matrix4x4& M, int *indx, double col[]) const
        {
            int i, ii = -1, ip,j;
            double sum;
            int N = 4;

            for(i = 0; i < N; i++){
                ip = indx[i];
                sum = col[ip];
                col[ip] = col[i];
                if(ii >= 0)
                    for(j = ii; j < i; j++)
                        sum -= M.row[i][j] * col[j];
                else if(sum)
                    ii = i;
                col[i] = sum;
            }

            for(i = N - 1; i >= 0; i--){
                sum = col[i];
                for(j = i + 1; j < N; j++)
                    sum -= M.row[i][j] * col[j];
                col[i] = sum / M.row[i][i];
            }
        }

    };


    /////////////////////////////////////////////////////////////////////////

    // Matrix op Matrix operations

    inline Matrix2x2 operator+(const Matrix2x2& m1, const Matrix2x2& m2)
    {
        int i;
        Matrix2x2 result;
        
        for(i = 0; i < 2; i++){
            result.row[i][0] = m1.row[i][0] + m2.row[i][0];
            result.row[i][1] = m1.row[i][1] + m2.row[i][1];
        }
        
        return result;
    }

    inline Matrix2x2 operator-(const Matrix2x2& m1, const Matrix2x2& m2)
    {
        int i;
        Matrix2x2 result;
        
        for(i = 0; i < 2; i++){
            result.row[i][0] = m1.row[i][0] - m2.row[i][0];
            result.row[i][1] = m1.row[i][1] - m2.row[i][1];
        }
        
        return result;
    }

    inline Matrix2x2 operator*(double a, const Matrix2x2& m)
    {
        int i;
        Matrix2x2 result;
        
        for(i = 0; i < 2; i++){
            result.row[i][0] = a * m.row[i][0];
            result.row[i][1] = a * m.row[i][1];
        }
        
        return result;
    }

    inline Matrix2x2 operator*(const Matrix2x2& m1, const Matrix2x2& m2)
    {
        int i, j, rc;
        Matrix2x2 result;
        
        for(i = 0; i < 2; i++)
            for(j = 0; j < 2; j++){
                result.row[i][j] = 0;
                for(rc = 0; rc < 2; rc++){
                    result.row[i][j] += m1.row[i][rc] * m2.row[rc][j];
                }
            }
            
            return result;
    }

    inline Matrix3x3 operator+(const Matrix3x3& m1, const Matrix3x3& m2)
    {
        int i, j;
        Matrix3x3 result;
        
        for(i = 0; i < 3; i++)
            for(j = 0; j < 3; j++)
                result.row[i][j] = m1.row[i][j] + m2.row[i][j];
            
            return result;
    }

    inline Matrix3x3 operator-(const Matrix3x3& m1, const Matrix3x3& m2)
    {
        int i, j;
        Matrix3x3 result;
        
        for(i = 0; i < 3; i++)
            for(j = 0; j < 3; j++)
                result.row[i][j] = m1.row[i][j] - m2.row[i][j];
            
            return result;
    }

    inline Matrix3x3 operator*(double a, const Matrix3x3& m)
    {
        Matrix3x3 result;
        int i, j;
        
        for(i = 0; i < 3; i++)
            for(j = 0; j < 3; j++)
                result.row[i][j] = a * m.row[i][j];
            
            return result;
    }

    inline Matrix3x3 operator*(const Matrix3x3& m1, const Matrix3x3& m2)
    {
        double h=m1.row[0][0];

        int i, j, rc;
        Matrix3x3 result;
        
        h=m1.row[0][0];
        for(i = 0; i < 3; i++){
            for(j = 0; j < 3; j++){
                result.row[i][j] = 0;
                for(rc = 0; rc < 3; rc++){
                    result.row[i][j] += m1.row[i][rc] * m2[rc][j];
                }
            }
        }
            
        return result;
    }

    inline Matrix4x4 operator+(const Matrix4x4& m1, const Matrix4x4& m2)
    {
        int i, j;
        Matrix4x4 result;
        
        for(i = 0; i < 4; i++)
            for(j = 0; j < 4; j++)
                result.row[i][j] = m1.row[i][j] + m2.row[i][j];
            
            return result;
    }

    inline Matrix4x4 operator-(const Matrix4x4& m1, const Matrix4x4& m2)
    {
        int i, j;
        Matrix4x4 result;
        
        for(i = 0; i < 4; i++)
            for(j = 0; j < 4; j++)
                result.row[i][j] = m1.row[i][j] - m2.row[i][j];
            
            return result;
    }

    inline Matrix4x4 operator*(double a, const Matrix4x4& m)
    {
        Matrix4x4 result;
        
        int i, j;
        for(i = 0; i < 4; i++)
            for(j = 0; j < 4; j++)
                result.row[i][j] = a * m.row[i][j];
            
            return result;
    }

    inline Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2)
    {
        int i, j, rc;
        Matrix4x4 result;
        
        for(i = 0; i < 4; i++)
            for(j = 0; j < 4; j++){
                result.row[i][j] = 0;
                for(rc = 0; rc < 4; rc++)
                    result.row[i][j] += m1.row[i][rc] * m2[rc][j];
            }
            
            return result;
    }

    /* Matrix-Vector Operations */

    inline Vector2d operator*(const Matrix2x2& m, const Vector2d& v)
    {
        int i, j;
        double sum;
        Vector2d result;
        
        for(i = 0; i < 2; i++){
            sum = 0;
            for(j = 0; j < 2; j++)
                sum += m.row[i][j] * v[j];
            result[i] = sum;
        }
        
        return result;
    }

    inline Vector3d operator*(const Matrix3x3& m, const Vector3d& v)
    {
        int i, j;
        double sum;
        Vector3d result;
        
        for(i = 0; i < 3; i++){
            sum = 0;
            for(j = 0; j < 3; j++)
                sum += m.row[i][j] * v[j];
            result[i] = sum;
        }
        
        return result;
    }

    inline Vector4d operator*(const Matrix4x4& m, const Vector4d& v)
    {
        int i, j;
        double sum;
        Vector4d result;
        
        for(i = 0; i < 4; i++){
            sum = 0;
            for(j = 0; j < 4; j++)
                sum += m.row[i][j] * v[j];
            result[i] = sum;
        }
        
        return result;
    }

    inline Vector2d operator*(const Vector2d& v, const Matrix2x2& m)
    {
        int i, j;
        double sum;
        Vector2d result;
        
        for(j = 0; j < 2; j++){
            sum = 0;
            for(i = 0; i < 2; i++)
                sum += v[i] * m.row[i][j];
            result[j] = sum;
        }
        
        return result;
    }

    inline Vector3d operator*(const Vector3d& v, const Matrix3x3& m)
    {
        int i, j;
        double sum;
        Vector3d result;
        
        for(j = 0; j < 3; j++){
            sum = 0;
            for(i = 0; i < 3; i++)
                sum += v[i] * m.row[i][j];
            result[j] = sum;
        }
        
        return result;
    }

    inline Vector4d operator*(const Vector4d& v, const Matrix4x4& m)
    {
        int i, j;
        double sum;
        Vector4d result;
        
        for(j = 0; j < 4; j++){
            sum = 0;
            for(i = 0; i < 4; i++)
                sum += v[i] * m.row[i][j];
            result[j] = sum;
        }
        
        return result;
    }

    // Outer product of v1, v2 (i.e. v1 times v2 transpose)
    inline Matrix2x2 operator&(const Vector2d& v1, const Vector2d& v2)
    {
        int i, j;
        Matrix2x2 product;
        
        for(i = 0; i < 2; i++)
            for(j = 0; j < 2; j++)
                product.row[i][j] = v1[i] * v2[j];
            
        return product;
    }

    inline Matrix3x3 operator&(const Vector3d& v1, const Vector3d& v2)
    {
        Matrix3x3 product;
        
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                product.row[i][j] = v1[i] * v2[j];
            
        return product;
    }

} //end of nprmlib namespace

#endif //_H_Matrix_H_

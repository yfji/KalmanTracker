#include "matrix.h"
#include <assert.h>
#include <math.h>

cv::Mat cholesky(const cv::Mat& A){
    assert(A.rows==A.cols);
    int dim=A.rows;
    cv::Mat L(dim, dim, CV_32F);

    float* data=(float*)A.data;
    float* tri_data=(float*)L.data;
    
    for(int k=0;k<dim;++k){
        float sum=0.0;
        for(int i=0;i<k;++i)
            sum+=tri_data[k*dim+i]*tri_data[k*dim+i];
        sum=data[k*dim+k]-sum;
        tri_data[k*dim+k]=sqrt(sum>=0?sum:0);
        for(int i= k+1; i<dim;++i){
            sum=0;
            for(int j=0;j<k;++j)
                sum+=tri_data[i*dim+j]*tri_data[k*dim+j];
            tri_data[i*dim+k]=(data[i*dim+k]-sum)/tri_data[k*dim+k];
        }
        for(int i=0;i<k;++i)
            tri_data[i*dim+k]=0;
    }
    return L;    //lower triangle
}

cv::Mat upper_triangle_inv(const cv::Mat& T){
    assert(T.rows==T.cols);
    int dim=T.rows;
    cv::Mat T_inv(dim, dim, CV_32F);

    float* data=(float*)T.data;
    float* tri_data=(float*)T_inv.data;

    for(int i=0;i<dim;++i)
        tri_data[i*dim+i]=1.0/data[i*dim+i];
    for(int i=dim-2;i>=0;--i){
        for(int j=i+1;j<dim;++j){
            float s=0;
            for(int k=i+1;k<=j;++k)
                s+=tri_data[k*dim+j]*data[i*dim+k];
            tri_data[i*dim+j]=-s/data[i*dim+i];
        }
    }
    return T_inv;
}

cv::Mat lower_triangle_inv(const cv::Mat& T){
    cv::Mat T_inv=upper_triangle_inv(T.t());
    return T.t();
}

cv::Mat cholesky_solve(const cv::Mat& A, const cv::Mat& B){
    cv::Mat L=cholesky(A);  //lower
    cv::Mat L_inv;
    cv::invert(L, L_inv, cv::DECOMP_LU);
    // cv::Mat L_inv=lower_triangle_inv(L);
    cv::Mat Y=L_inv*B;
    cv::Mat X=L_inv.t()*Y;
    return X;
}
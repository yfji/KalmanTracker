#ifndef _MATRIX_
#define _MATRIX_

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;

cv::Mat cholesky(const cv::Mat& A);
cv::Mat upper_triangle_inv(const cv::Mat& T);
cv::Mat lower_triangle_inv(const cv::Mat& T);
cv::Mat cholesky_solve(const cv::Mat& A, const cv::Mat& B);

#endif
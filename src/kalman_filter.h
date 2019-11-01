#ifndef _KALMAN_FILTER_
#define _KALMAN_FILTER_

#include "matrix.h"

#define STATEDIM    8
#define MEASUREDIM  4

class KF{
public:
    KF();
    ~KF();

    void init(cv::Mat& measurement);
    cv::Mat predict();
    void update(cv::Mat& measurement);
    void project();

private:
    cv::Mat _state_mat;
    cv::Mat _observe_mat;
    cv::Mat _state;
    cv::Mat _covariance;
    cv::Mat Q;  //system error
    cv::Mat R;  //measure error

    cv::Mat _state_dash;
    cv::Mat _covariance_dash;

    cv::Mat _state_proj;
    cv::Mat _covariance_proj;
};

#endif
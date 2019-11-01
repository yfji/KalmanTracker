#include "kalman_filter.h"

KF::KF(){
    _state_mat = cv::Mat::eye(STATEDIM, STATEDIM, CV_32F);
    for(int i = 0; i < MEASUREDIM; i++){
        _state_mat.at<float>(i, MEASUREDIM + i) = 1.0;
    }
    _observe_mat = cv::Mat::eye(MEASUREDIM, STATEDIM, CV_32F);
    _covariance=cv::Mat::zeros(STATEDIM, STATEDIM, CV_32F);
    Q=cv::Mat::zeros(STATEDIM, STATEDIM, CV_32F);
    R=cv::Mat::zeros(MEASUREDIM, MEASUREDIM, CV_32F);
}

KF::~KF(){

}

void KF::init(cv::Mat& measurement){
    _state=cv::Mat::zeros(STATEDIM, 1, CV_32F);
    cv::setIdentity(_covariance, cv::Scalar::all(1));

    cv::setIdentity(Q, cv::Scalar::all(1e-5));
    cv::setIdentity(R, cv::Scalar::all(1e-1));

    for(int i=0;i<MEASUREDIM;++i)
        _state.at<float>(i)=measurement.at<float>(i);
}

cv::Mat KF::predict(){
    _state_dash=_state_mat*_state;
    _covariance_dash=_state_mat*_covariance*_state_mat.t()+Q;

    return _state_dash;
}

void KF::project(){
    _state_proj=_observe_mat*_state_dash;
    cv::Mat cov=_observe_mat*_covariance_dash*_observe_mat.t();
    _covariance_proj=cov+R;
}

void KF::update(cv::Mat& measurement){
    project();
    cv::Mat M=_covariance_dash*_observe_mat.t();
    cv::Mat kalman_gain=cholesky_solve(_covariance_proj, M.t()).t();
    
    // cv::Mat cov_inv;
    // cv::invert(_covariance_proj, cov_inv, cv::DECOMP_CHOLESKY);
    // cv::Mat kalman_gain=M*cov_inv;
    cv::Mat innovation=measurement-_state_proj;

    _state=_state_dash+kalman_gain*innovation;
    _covariance=_covariance_dash-kalman_gain*_observe_mat*_covariance_dash;
}
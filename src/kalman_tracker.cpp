#include "kalman_tracker.h"


KalmanTracker::KalmanTracker(){
    kf.reset(new KF());
    // kf.reset(new cv::KalmanFilter(STATEDIM, MEASUREDIM));    
}

KalmanTracker::~KalmanTracker(){

}

void KalmanTracker::init(cv::Rect &roi){
    float data[]={
        roi.x+roi.width*0.5,roi.y+roi.height*0.5,1.0*roi.width/roi.height,roi.height
    };
    cv::Mat measure(MEASUREDIM,1,CV_32F,data);
    kf->init(measure);
    // cv::setIdentity(kf->transitionMatrix);
    // cv::setIdentity(kf->measurementMatrix);
    // cv::setIdentity(kf->processNoiseCov, cv::Scalar::all(1e-5));
    // cv::setIdentity(kf->measurementNoiseCov, cv::Scalar::all(1e-1));
    // cv::setIdentity(kf->errorCovPost, cv::Scalar::all(1)); 
    // for(int i = 0; i < MEASUREDIM; i++){
    //     kf->transitionMatrix.at<float>(i, MEASUREDIM + i) = 1.0;
    // }
    // // randn(kf->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
    // kf->statePost.setTo(0);
    // for(int i=0;i<MEASUREDIM;++i){
    //     kf->statePost.at<float>(i)=data[i];
    // }
}

cv::Rect KalmanTracker::predict(){
    cv::Rect pred_roi;
    cv::Mat pred=kf->predict();
    cout<<"pred"<<endl;

    float* pred_data=(float*)pred.data;
    pred_roi.height=int(pred_data[3]);
    pred_roi.width=int(pred_data[3]*pred_data[2]);
    pred_roi.x=int(pred_data[0]-pred_roi.width*0.5);
    pred_roi.y=int(pred_data[1]-pred_roi.height*0.5);

    return pred_roi;
}

void KalmanTracker::update(cv::Rect& roi){
    float data[]={
        roi.x+roi.width*0.5,roi.y+roi.height*0.5,1.0*roi.width/roi.height,roi.height
    };
    cv::Mat measure(MEASUREDIM,1,CV_32F,data);
    kf->update(measure);
    cout<<"update"<<endl;
}
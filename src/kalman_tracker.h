#ifndef _KALMAN_TRACKER_
#define _KALMAN_TRACKER_
#include "kalman_filter.h"
#include <memory>
#include <opencv2/opencv.hpp>

class KalmanTracker
{
public:
    KalmanTracker();
	~KalmanTracker();

	void init(cv::Rect &roi);
	void update(cv::Rect& roi);
    cv::Rect predict();

    
private:
    const int stateDim=STATEDIM;
    const int measureDim=MEASUREDIM;

    std::shared_ptr<KF> kf;
    // std::shared_ptr<cv::KalmanFilter> kf;
};

#endif
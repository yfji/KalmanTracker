#include "kalman_filter.h"
#include "kalman_tracker.h"
#include <time.h>

int center_x;
int center_y;
const int roi_width=50;
const int roi_height=50;
cv::Rect pred_roi;
bool started=false;
cv::Mat canvas;
int cnt=0;

cv::Rect getRect(const cv::Point2i &pt){
    cv::RNG rng(time(NULL));
    float offset_x=rng.gaussian(10);
    float offset_y=rng.gaussian(10);
    int height=(int)(20*offset_y)+roi_height;
    int width=(int)(20*offset_x)+roi_width;
    return cv::Rect(pt.x-width/2,pt.y-height/2,width,height);
}

void on_mouse(int event,int x,int y,int flags,void *ustc){
    if(event==cv::EVENT_LBUTTONDOWN){
        canvas.setTo(0);
        started=!started;
        if(started){
            cout<<"Kalman tracker init"<<endl;
            KalmanTracker* tracker=(KalmanTracker*)ustc;
            cv::Rect roi=getRect(cv::Point2i(x,y));
            tracker->init(roi);
            cv::rectangle(canvas, roi, cv::Scalar(0,0,255), 2);
        }
    }
    if (event == cv::EVENT_MOUSEMOVE)
    {
        if(!started)
            return;
        canvas.setTo(0);
        // cout<<"Kalman tracker predict and update"<<endl;
        cv::Rect roi=getRect(cv::Point2i(x,y));
        KalmanTracker* tracker=(KalmanTracker*)ustc;
        pred_roi=tracker->predict();
        cv::rectangle(canvas, pred_roi, cv::Scalar(0,255,255), 2);
        cv::rectangle(canvas, roi, cv::Scalar(0,0,255), 1);
        cout<<pred_roi<<endl;
        tracker->update(roi);
    }
}

int main(){
    KalmanTracker tracker;
    canvas.create(cv::Size(640,640), CV_8UC3);
    canvas.setTo(0);
    cv::namedWindow("kalman");
    cv::setMouseCallback("kalman", on_mouse, (void*)(&tracker));

    while(1){
        cv::imshow("kalman", canvas);
        if(cv::waitKey(10)==27)
            return 0;
    }
}
#ifndef TEMPTHREAD_H
#define TEMPTHREAD_H

#include <QThread>
#include <QDebug>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include "opencv2/objdetect.hpp"
#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define TEMP_WINDOW_NAME "Taking temperature ..."

class tempThread : public QThread
{
    Q_OBJECT
public:
    tempThread();

protected:
    void run() override;

private:
    static void imageCb(const sensor_msgs::Image::ConstPtr& msg);
    //void thermalCb(const sensor_msgs::ImageConstPtr& msg);
    //cv::Mat image, thermal;
    cv::CascadeClassifier face_cascade, eye_cascade;
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    static cv::Mat imag;
    static bool get_img;

};

#endif // TEMPTHREAD_H

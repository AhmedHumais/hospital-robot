/**
 * @file tempthread.cpp
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief source file for thread implementing temperature measurement feature
 * @class tempThread
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

#include "tempthread.h"

cv::Mat tempThread::imag;
cv::Mat tempThread::thermal;
bool tempThread::get_img = true;
bool tempThread::get_therm = true;

tempThread::tempThread(){

}

void tempThread::imageCb(const sensor_msgs::Image::ConstPtr& msg){
    if(get_img){
//        qDebug()<< "Storing image";
        int sz = (msg->step)*(msg->height);
        char* x = new char[sz];
        for(int i=0;i<sz; i++){
            x[i] = msg->data[i];
        }
        cv::Mat rcv_img((int)msg->height, (int)msg->width, CV_8UC3, x, msg->step);
        cv::cvtColor(rcv_img, imag, cv::COLOR_RGB2BGR);

        get_img = false;
    }
}

void tempThread::thermalCb(const sensor_msgs::Image::ConstPtr& msg){
    if(get_therm){
      //  qDebug()<< "Storing image";
        int sz = (msg->step)*(msg->height);
        char* x = new char[sz];
        for(int i=0;i<sz; i++){
            x[i] = msg->data[i];
        }
        thermal = cv::Mat((int)msg->height, (int)msg->width, CV_16UC1, x, msg->step);

        get_therm = false;
        // int sz = (msg->step)*(msg->height);
        // char* x = new char[sz];
        // for(int i=0;i<sz; i++){
        //     x[i] = msg->data[i];
        // }
        // cv::Mat rcv_img((int)msg->height, (int)msg->width, CV_8UC3, x, msg->step);
        // cv::cvtColor(rcv_img, thermal, cv::COLOR_RGB2BGR);

        // get_therm = false;

    }
}

void tempThread::run()
{
    cv::namedWindow(TEMP_WINDOW_NAME);
    image_sub_ = nh_.subscribe("optris/visible_image_view", 1, imageCb);
    thermal_sub_ = nh_.subscribe("optris/thermal_image", 1, thermalCb);
    face_cascade.load("/home/ahmed/opencv_build/opencv/data/haarcascades/haarcascade_frontalface_alt.xml");
    eye_cascade.load("/home/ahmed/opencv_build/opencv/data/haarcascades/haarcascade_eye.xml");
    ros::Rate rt(5);
    std::vector<cv::Rect> faces;
    cv::Rect sel;
    cv::Mat gray;
    while(ros::ok()){
        if(isInterruptionRequested()){
            cv::destroyWindow(TEMP_WINDOW_NAME);
            break;
        }

        if(!imag.empty() && !thermal.empty()){

            cv::cvtColor( imag, gray, cv::COLOR_BGR2GRAY );
            cv::equalizeHist( gray, gray );
            //-- Detect faces
//            qDebug()<<"detecting";
            face_cascade.detectMultiScale( gray, faces );
//            qDebug()<< "done";
            if(!faces.empty()){
//                qDebug() << "detected face";
//                cv::rectangle(imag, faces[0], cv::Scalar(255,0,0));
                cv::Mat faceROI = gray( faces[0] );
                std::vector<cv::Rect> eyes;
                eye_cascade.detectMultiScale( faceROI, eyes );
                if(eyes.size() == 2){
//                    qDebug() << "detected eyes";
                    int y1 = eyes[0].y;
                    int y2 = eyes[1].y;
                    int x_c1 = eyes[0].x + eyes[0].width*0.5;
                    int x_c2 = eyes[1].x + eyes[1].width*0.5;
                    int y = (y1 < y2? y1 : y2);
                    int w = abs(x_c1 - x_c2);
                    if(y > 8 && w > 4){
                        cv::Rect forehead(faces[0].x + (x_c1 < x_c2? x_c1 : x_c2), faces[0].y + 0.5*y, w, y/2);
                        cv::rectangle(imag, forehead, cv::Scalar(255,0,0));
                        float c1 = 21.0/w, c2 = 21.0/w;
                        int x_sc = c1*(forehead.x - 320) + 80;
                        int y_sc = c2*(forehead.y - 240) + 60 + (exp(50.0/(w-30))-3);
                        cv::Rect f_sc(x_sc, y_sc, c1*w, c2*forehead.height);
                        // qDebug() << "rect: " << f_sc.x << ", " << f_sc.y << ", " << f_sc.width << ", " << f_sc.height ;
                        // cv::rectangle(thermal, f_sc, cv::Scalar(255,0,0));
                        cv::Scalar Temperature = cv::mean(thermal(f_sc));
                        float temperature = (float)(Temperature[0]-1000)/10.0;
                        char buf[40];
                        sprintf(buf, "Temperature: %.2f", temperature);
                        cv::putText(imag, buf, cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,250), 1);
                        qDebug() << "detected forehead";
                    }
                }
            }
            else
                qDebug()<<"not detected";
            cv::imshow(TEMP_WINDOW_NAME, imag);
            get_img = true;
            get_therm = true;
        }
        rt.sleep();
        ros::spinOnce();
    }
}

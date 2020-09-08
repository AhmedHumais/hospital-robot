#include "tempthread.h"

cv::Mat tempThread::imag;
bool tempThread::get_img = true;

tempThread::tempThread(){
    face_cascade.load("/home/ahmed/opencv_build/opencv/data/haarcascades/haarcascade_frontalface_alt.xml");
    eye_cascade.load("/home/ahmed/opencv_build/opencv/data/haarcascades/haarcascade_eye.xml");
}

void tempThread::imageCb(const sensor_msgs::Image::ConstPtr& msg){
    //char* x = (msg->data).data();
    if(get_img){
        qDebug()<< "Storing image";
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

void tempThread::run()
{
    image_sub_ = nh_.subscribe("camera/color/image_raw", 1, imageCb);
    ros::Rate rt(5);
    std::vector<cv::Rect> faces;
    cv::Mat gray;
    cv::namedWindow(TEMP_WINDOW_NAME);
    while(ros::ok()){
        if(!imag.empty()){
            cv::cvtColor( imag, gray, cv::COLOR_BGR2GRAY );
            cv::equalizeHist( gray, gray );
            //-- Detect faces
            qDebug()<<"detecting";
            face_cascade.detectMultiScale( gray, faces );
            if(!faces.empty()){
                cv::Mat faceROI = gray( faces[0] );
                std::vector<cv::Rect> eyes;
                eye_cascade.detectMultiScale( faceROI, eyes );
                if(eyes.size() == 2){
                    int y1 = eyes[0].y;
                    int y2 = eyes[1].y;
                    int x_c1 = eyes[0].x + eyes[0].width*0.5;
                    int x_c2 = eyes[1].x + eyes[1].width*0.5;
                    int y = (y1 < y2? y1 : y2);
                    int w = abs(x_c1 - x_c2);
                    if(y > 8 && w > 4){
                        cv::Rect forehead(faces[0].x + (x_c1 < x_c2? x_c1 : x_c2), faces[0].y + y/4, w, y/2);
                        cv::rectangle(imag, forehead, cv::Scalar(255,0,0));
                        cv::imshow(TEMP_WINDOW_NAME, imag);
                        //cv::waitKey(3);
                        qDebug() << "detected forehead";
                    }
                }
            }
            else
                qDebug()<<"not detected";
            get_img = true;
        }
        qDebug()<< "loop iteration";
        rt.sleep();
        ros::spinOnce();
    }
}

#include "emgthread.h"

emgThread::emgThread(){
    emg_stop_pub = nh_.advertise<std_msgs::Bool>("e_stop", 1);

}

void emgThread::set_emg_state(bool state){
    this->emg_state = state;
}

void emgThread::run(){
    std_msgs::Bool msg;
    msg.data = this->emg_state;

    emg_stop_pub.publish(msg);
    qDebug()<< "sent smergency messgae";
}

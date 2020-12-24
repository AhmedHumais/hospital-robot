/**
 * @file emgthread.cpp
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief source file for the thread implementing emergency stop function
 * @class rmgThread
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

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

/**
 * @file statusthread.cpp
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief source file for thread implementing status reporting function
 * @class statusThread
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

#include "statusthread.h"

float statusThread::batt_c = 0;
float statusThread::batt_v = 21.0;
ros::Time statusThread::last_update;

void statusThread::status_cb(const jackal_msgs::Status::ConstPtr& msg)
{
    batt_v = msg->measured_battery;
    batt_c = msg->total_current;
    last_update = ros::Time::now();
}

statusThread::statusThread()
{
    status_sub_ = nh_.subscribe<jackal_msgs::Status>("status", 5, status_cb);
}

void statusThread::run()
{
    ros::Rate loop_rate(1);
    QString conn;
    while(ros::ok() && !(this->isInterruptionRequested()))
    {
        if((ros::Time::now() - last_update) > ros::Duration(3))
            conn = "N/A";
        else
            conn = "OK";
        int bat_cap = (int)(batt_v - 21.0)/0.084;
        float bat_pow = batt_v * batt_c;
        char b_cap[5]; char b_pow[10];
        sprintf(b_cap, "%d %%", bat_cap);
        sprintf(b_pow, "%.2f W", bat_pow);

        QString battery(b_cap); QString power(b_pow);
        emit update_status(conn, battery, power);

        loop_rate.sleep();
        ros::spinOnce();
    }
}

/**
 * @file statusthread.h
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief header file for thread implementing status reporting function
 * @class statusThread
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

#ifndef STATUSTHREAD_H
#define STATUSTHREAD_H

#include <QThread>
#include <QString>
#include "ros/ros.h"
#include "jackal_msgs/Status.h"

class statusThread : public QThread
{
    Q_OBJECT
public:
    statusThread();


signals:
    void update_status( QString conn, QString voltage, QString power);

protected:
    void run() override;

private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;
    static float batt_v, batt_c;
    static ros::Time last_update;
    static void status_cb(const jackal_msgs::Status::ConstPtr& msg);
};

#endif // STATUSTHREAD_H

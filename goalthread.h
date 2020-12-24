/**
 * @file goalthread.h
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief header file for sending goals using a separate thread
 * @class goalThread
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

#ifndef GOALTHREAD_H
#define GOALTHREAD_H

#include <QThread>
#include <QString>
#include <vector>
#include "ros/ros.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"

class goalThread : public QThread
{
    Q_OBJECT
public:
    goalThread();
    void enable(bool state);
    QAtomicInt cancel_, enable_;
signals:
    void auto_status( QString stat);

protected:
    void run() override;

private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;
    ros::Publisher cancel_pub_;
    static QString status_;
    static void status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
};


#endif // GOALTHREAD_H

/**
 * @file rosthread.h
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief header file for thread implementing teleoperation
 * @class rosThread
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <QThread>
#include <QDebug>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <QAtomicInt>
#include <sstream>

//const int max_trans_speed = 10; // value*0.1 m/s
//const int max_rot_speed = 10; // value*0.1 rad/s

class rosThread : public QThread
{
    Q_OBJECT
public:
    rosThread();
    QAtomicInt fwd, rgt, lft, bwd, stp, enbl;
    QAtomicInt max_trans_speed = 6, max_rot_speed = 10;

protected:
    void run() override;

private:
    int cmd_x, cmd_y, prev_x, prev_y;
    int get_cmd(int prev, int now);    
};

#endif // ROSTHREAD_H

#ifndef EMGTHREAD_H
#define EMGTHREAD_H

#include <QThread>
#include <QDebug>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

class emgThread : public QThread
{
    Q_OBJECT
public:
    emgThread();
    void set_emg_state(bool);

protected:
    void run() override;

private:
    ros::NodeHandle nh_;
    ros::Publisher emg_stop_pub;
    bool emg_state = true;
};

#endif // EMGTHREAD_H

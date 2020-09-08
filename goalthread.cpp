#include "goalthread.h"

QString goalThread::status_ = "waiting";

void goalThread::status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    std::vector<actionlib_msgs::GoalStatus> stat_list = msg->status_list;
    if (!stat_list.empty())
    {
//        int sz = sizeof(stat_list)/sizeof(stat_list[0]);
        uint8_t stat = stat_list.back().status;
        switch(stat_list.back().status)
        {
        case actionlib_msgs::GoalStatus::PENDING:
            status_ = "received";
            break;
        case actionlib_msgs::GoalStatus::ACTIVE:
            status_ = "Going";
            break;
        case actionlib_msgs::GoalStatus::PREEMPTED:
        case actionlib_msgs::GoalStatus::RECALLED:
            status_ = "Cancelled";
            break;
        case actionlib_msgs::GoalStatus::SUCCEEDED:
            status_ = "Reached";
            break;
        case actionlib_msgs::GoalStatus::ABORTED:
        case actionlib_msgs::GoalStatus::REJECTED:
            status_ = "Unreachable";
            break;
        case actionlib_msgs::GoalStatus::PREEMPTING:
        case actionlib_msgs::GoalStatus::RECALLING:
            status_ = "Cancelling";
            break;
        }
    }
    else
    {
        status_ = "waiting";
    }
}

goalThread::goalThread(): cancel_(0), enable_(0)
{
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 2);
}

void goalThread::run()
{
    status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 5, status_cb);
    ros::Rate rt_(5);
    actionlib_msgs::GoalID goal_;
    while(ros::ok() && !(this->isInterruptionRequested()))
    {
        if (cancel_ == 1){
            goal_.stamp = ros::Time::now();
            cancel_pub_.publish(goal_);
            cancel_ = 0;
        }
        if (enable_){
            emit auto_status(status_);
        }
        rt_.sleep();
        ros::spinOnce();
    }

}

#include "rosthread.h"

rosThread::rosThread():cmd_x(0), cmd_y(0), enbl(1)
{

}

void rosThread::run()
{
    ros::NodeHandle n;
    ros::Publisher mov_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(10);
    geometry_msgs::Twist mov_msg;

    while(ros::ok()){

        if(this->stp){
            cmd_x = 0;
            cmd_y = 0;
            this->stp.store(0);
        }

        if(this->fwd){
            cmd_x = get_cmd(cmd_x, max_trans_speed);
        }
        else if(this->bwd){
            cmd_x = get_cmd(cmd_x, -max_trans_speed);
        }
        else{
            cmd_x = get_cmd(cmd_x, 0);
        }

        if(this->rgt){
            cmd_y = -5;
        }
        else if(this->lft){
            cmd_y = 5;
        }
        else{
            cmd_y = get_cmd(cmd_y, 0);
        }

        if(this->isInterruptionRequested())
        {
            mov_msg.linear.x = 0;
            mov_msg.angular.z = 0;
            mov_pub.publish(mov_msg);
            break;
        }

        mov_msg.linear.x = double(cmd_x)/10.0;
        mov_msg.angular.z = double(cmd_y)/10.0;
        if (this->enbl){
            if(prev_x!=0 || cmd_x!=0 || prev_y!=0 || cmd_y!=0)
                mov_pub.publish(mov_msg);
        }
        prev_x = cmd_x;
        prev_y = cmd_y;

        loop_rate.sleep();
        ros::spinOnce();
    }
}

int rosThread::get_cmd(int prev, int now){
    int err = now - prev;

    if(err > 0){
        return prev + 1;
    }
    if(err < 0){
        return prev - 1;
    }
    return prev;
}

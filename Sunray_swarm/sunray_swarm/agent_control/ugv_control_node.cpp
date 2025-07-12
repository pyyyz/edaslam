#include <ros/ros.h>
#include <signal.h>

#include "ugv_control.h"

void mySigintHandler(int sig)
{
    ROS_INFO("[ugv_control_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_control_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 控制器
    UGV_CONTROL ugv_control;
    ugv_control.init(nh);

    ros::spinOnce();
    ros::Duration(1.0).sleep();


    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        ugv_control.mainloop();
        // sleep
        rate.sleep();
    }

    return 0;
}

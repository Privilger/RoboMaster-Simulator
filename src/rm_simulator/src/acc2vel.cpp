#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include "roborts_msgs/TwistAccel.h"
#include <iostream>


ros::Publisher  cmd_pub;

void Callback(const roborts_msgs::TwistAccel::ConstPtr& msg)
{
    geometry_msgs::Twist cmd;
    cmd = msg->twist;
    cmd_pub.publish(cmd);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc2vel");

    ros::NodeHandle nh;

    ros::Subscriber acc_sub = nh.subscribe("cmd_vel_acc", 100, &Callback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();
    return 0;
}

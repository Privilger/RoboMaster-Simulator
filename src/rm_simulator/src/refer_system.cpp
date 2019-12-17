#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "rm_simulator/ReferSystemInfo.h"


class ReferSystem
{
public:
    ReferSystem(ros::NodeHandle& nodehandle) :
    nh_(nodehandle)    
    {
        state_pub = nh_.advertise<rm_simulator::ReferSystemInfo>("refer_system_info", 1, true);
        refer_system_msg.header.frame_id = "/map";

        for (int i = 0; i < 4; ++i) {
            shoot_sub[i] = nh_.subscribe("/jackal"+std::to_string(i)+"/shoot", 10, &ReferSystem::updateShoot, this);
        }
    }
    void updateShoot(const std_msgs::String::ConstPtr msg){
        // data formate: a to b
        if(msg->data.length() == 4){
            refer_system_msg.remain_bullet[msg->data[0]-'0'] --;

            //TODO 血量扣除，加上前后左右的orientation信息  Using robotPose to calc the different damage.
            refer_system_msg.robotHealth[msg->data[3]-'0'] -= 20;
        }
    }

    void game_spin(){
        try{
            for (int i = 0; i < 4; ++i) {
                listener.lookupTransform("/map", "/jackal"+std::to_string(i)+"/base_link",
                                         ros::Time(0), robot_status[i]);
            }
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        refer_system_msg.header.stamp = ros::Time(0);

        for (int i = 0; i < 4; ++i) {
            refer_system_msg.robotPose[i].x = robot_status[i].getOrigin().x();
            refer_system_msg.robotPose[i].y = robot_status[i].getOrigin().y();
            refer_system_msg.robotPose[i].yaw = tf::getYaw(robot_status[i].getRotation());
            refer_system_msg.gameTime = ros::Time::now().toSec();
        }

        // get callback data
        ros::spinOnce();

        // Set cellState

    }



private:
    ros::NodeHandle                 nh_;
    ros::Publisher                  state_pub;
    ros::Subscriber                 shoot_sub[4];
    tf::TransformListener           listener;
    rm_simulator::ReferSystemInfo   refer_system_msg;
    tf::StampedTransform            robot_status[4];
    double                          current_secs;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "simulate_refer_system_node");
    ros::NodeHandle node_handle;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    ReferSystem referSystem(node_handle);

    ros::Rate loop_rate(120);

    while (ros::ok()){
        referSystem.game_spin();
        loop_rate.sleep();

    }
    return 0;
}

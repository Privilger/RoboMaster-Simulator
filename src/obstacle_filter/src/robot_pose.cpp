//
// Created by lima on 2020/1/1.
//

#include "../include/obstacle_filter/robot_pose.h"

robot_pose::robot_pose(tf::TransformListener& tf,std::string jackal_ns):tf_(tf),jackal_ns_(jackal_ns){
    global_frame_="/map";
    if(jackal_ns_.find("jackal0")!=std::string::npos)
        robot_base_frame_="jackal1/base_link";
    if(jackal_ns_.find("jackal1")!=std::string::npos)
        robot_base_frame_="jackal0/base_link";
    if(jackal_ns_.find("jackal2")!=std::string::npos)
        robot_base_frame_="jackal3/base_link";
    if(jackal_ns_.find("jackal3")!=std::string::npos)
        robot_base_frame_="jackal2/base_link";

    std::string tf_error;
    ros::Time last_error = ros::Time::now();
    while (ros::ok() && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), \
         ros::Duration(0.01), &tf_error)) {
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now()) {
            last_error = ros::Time::now();
        }
        tf_error.clear();
    }
}
robot_pose::~robot_pose(){

}
bool robot_pose::GetRobotPose(tf::Stamped<tf::Pose> &global_pose) const{
    global_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now();
    try {
        tf_.transformPose(global_frame_, robot_pose, global_pose);
    }
    catch (tf::LookupException &ex) {
        ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
        return false;
    }
    catch (tf::ConnectivityException &ex) {
        ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException &ex) {
        ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
        return false;
    }
    return true;
}

bool robot_pose::GetRobotPose(geometry_msgs::PoseStamped &global_pose) const{
    tf::Stamped<tf::Pose> tf_global_pose;
    if (GetRobotPose(tf_global_pose)) {
        tf::poseStampedTFToMsg(tf_global_pose, global_pose);
        return true;
    } else {
        return false;
    }
}
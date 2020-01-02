//
// Created by lima on 2020/1/1.
//

#ifndef RMAI_WS_ROBOT_POSE_H
#define RMAI_WS_ROBOT_POSE_H

#include <tf/transform_listener.h>
class robot_pose {
public:
    robot_pose(tf::TransformListener& tf,std::string jackal_ns);
    ~robot_pose();
    bool GetRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
    bool GetRobotPose(geometry_msgs::PoseStamped & global_pose) const;
protected:
    tf::TransformListener& tf_;
    std::string global_frame_, robot_base_frame_,jackal_ns_;
};


#endif //RMAI_WS_ROBOT_POSE_H

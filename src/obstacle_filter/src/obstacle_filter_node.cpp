#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "obstacle_detector/Obstacles.h"
#include "obstacle_filter/arena_filter.h"
#include <cmath>
#include "../include/obstacle_filter/robot_pose.h"

obstacle_detector::Obstacles filter_msg;
geometry_msgs::PoseStamped current_start;



void cmdVelCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{
  filter_msg.circles.clear();
  filter_msg.segments.clear();
  filter_msg.header.frame_id = "map";
  for(auto it_c = msg->circles.begin() ; it_c!=msg->circles.end(); it_c++){
    Point p(it_c->center.x, it_c->center.y);
    if(abs(current_start.pose.position.x-it_c->center.x)<0.25&&abs(current_start.pose.position.y-it_c->center.y)<0.25){
        continue;
    }
    if (IsPointInAerna(p, 0.25))
    {
      continue;
    }
    else
    {
      filter_msg.circles.push_back(*it_c);
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_filter_node");
  ros::NodeHandle nh("~");
  std::string my_color;
  nh.param<std::string>("color", my_color, "green");
  std::cout<<my_color<<std::endl;
  std::string jackal_ns;
  ros::param::get("~jackal_ns",jackal_ns);
  ROS_INFO("!!!!!!!!!!!obstacle_filter jackal_ns:%s",jackal_ns.c_str());
  ros::Subscriber sub = nh.subscribe("/"+jackal_ns+"/raw_obstacles", 1000, &cmdVelCallback);
  ros::Publisher  obstacle_filter_pub = nh.advertise<obstacle_detector::Obstacles>("/"+jackal_ns+"/obstacle_filtered", 1000);

  ros::Rate loop_rate(40);
  std::shared_ptr <tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  robot_pose get_robot_pose(*tf_ptr_,jackal_ns);
  while (ros::ok()){
    get_robot_pose.GetRobotPose(current_start);
//      ROS_INFO("!!!!!!!!!!!obstacle_filter x,y:%f %f",current_start.pose.position.x,current_start.pose.position.y);
    filter_msg.header.stamp = ros::Time::now();
    obstacle_filter_pub.publish(filter_msg);  

    ros::spinOnce();    
    loop_rate.sleep();
  } 

  return 0;
}

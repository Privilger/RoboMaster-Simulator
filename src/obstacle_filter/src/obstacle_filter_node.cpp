#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "obstacle_detector/Obstacles.h"
#include "obstacle_filter/arena_filter.h"
#include <cmath>


obstacle_detector::Obstacles filter_msg;

void cmdVelCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{
	filter_msg.circles.clear();
	filter_msg.segments.clear();
	filter_msg.header.frame_id = "map";

	for(auto it_c = msg->circles.begin() ; it_c!=msg->circles.end(); it_c++){
		Point p(it_c->center.x, it_c->center.y); 
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

  ros::Subscriber sub = nh.subscribe("/obstacles", 1000, &cmdVelCallback);
  ros::Publisher  obstacle_filter_pub = nh.advertise<obstacle_detector::Obstacles>("/obstacle_filtered", 1000);

  ros::Rate loop_rate(40);

  
  while(ros::ok()){

    filter_msg.header.stamp = ros::Time::now();
    obstacle_filter_pub.publish(filter_msg);  

    ros::spinOnce();    
    loop_rate.sleep();
  } 

  return 0;
}

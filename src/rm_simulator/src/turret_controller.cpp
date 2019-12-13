#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>

class TurretPositionController
{
public:
    TurretPositionController(ros::NodeHandle& nodehandle) :
    nh_(nodehandle)    
    {        
        turret_position_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("turret_position_controller/command", 1, true);
        turret_position_sub = nh_.subscribe("turret_position", 10, &TurretPositionController::positionCallback, this);
        traj.header.frame_id = "front_camera_mount";
        traj.joint_names.resize(1);
        traj.points.resize(1);
        traj.points[0].positions.resize(1);
        traj.points[0].effort.resize(1);
        traj.joint_names[0] ="front_camera_pivot_joint";
    }

    void positionCallback(const std_msgs::Float32::ConstPtr& msg){
        traj.header.stamp = ros::Time::now();       
        traj.points[0].positions[0] = msg->data;
        traj.points[0].effort[0] = 100;        
        traj.points[0].time_from_start = ros::Duration(0.5);
        turret_position_pub.publish(traj);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  turret_position_pub;
    ros::Subscriber turret_position_sub;
    trajectory_msgs::JointTrajectory traj;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "turret_position_controller_node");
    ros::NodeHandle node_handle;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    TurretPositionController controller(node_handle);

    ros::spin();
    return 0;
}

#include "ros/ros.h"
#include "rm_simulator/GameState.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_game_node");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rm_simulator::GameState>("start_game");
    rm_simulator::GameState srv;
    srv.request.start = true;
    if (client.call(srv))
    {
        if(srv.response.received==true){ROS_INFO_STREAM("received");}
    }
    else
    {
        ROS_ERROR("Failed to call service start_game");
        return 1;
    }

    return 0;
}

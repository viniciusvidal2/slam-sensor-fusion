#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>

#include <ros/ros.h>

#include "localization/obstacle_point_cloud_generator_node.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_point_cloud_generator_node");
    ros::NodeHandle nh;
    ObstaclePointCloudGeneratorNode node(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}

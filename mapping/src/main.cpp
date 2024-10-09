#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <ros/ros.h>

#include "mapping/map_data_save_node.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "map_data_save_node");
    ros::NodeHandle nh("~");
    std::shared_ptr<MapDataSaver> node = std::make_shared<MapDataSaver>(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}

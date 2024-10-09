#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>

#include <ros/ros.h>

#include "localization/localization_node.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh("~")
    std::shared_ptr<LocalizationNode> node = std::make_shared<LocalizationNode>(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

#include "mapping/map_data_save_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapDataSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

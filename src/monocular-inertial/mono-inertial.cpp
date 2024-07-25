#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;

    auto node = std::make_shared<MonocularInertialNode>();
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

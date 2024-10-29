//
// Created by shibo zhao on 2020-09-27.
//
#include "rclcpp/rclcpp.hpp"
#include "super_odometry/ScanRegistration/scanRegistration.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    options.arguments({"scan_registration_node"});
    
    std::shared_ptr<superodom::scanRegistration> scanRegistration =
        std::make_shared<superodom::scanRegistration>(options);

    scanRegistration->imuBuf.allocate(5000);
    scanRegistration->visualOdomBuf.allocate(200);
    scanRegistration->lidarBuf.allocate(50);
    scanRegistration->initInterface();

    rclcpp::spin(scanRegistration->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

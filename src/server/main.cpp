#include <iostream>
#include <memory>

#include "server/server.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<Server::MultibotServer>();
    server->loadInstances();

    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}
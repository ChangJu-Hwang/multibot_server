#include <iostream>
#include <memory>
#include <future>
#include <thread>

#include "multibot_server/server.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<Server::MultibotServer>();
    
    auto spinThread = std::async(
        [server]()
        {
            rclcpp::spin(server);
            rclcpp::shutdown();
        });

    auto panelThread = std::async(
        [&argc, &argv, server]()
        {
            server->execServerPanel(argc, argv);
        });

    spinThread.get();
    panelThread.get();

    return 0;
}
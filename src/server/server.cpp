#include "server/server.hpp"

using namespace Server;
using namespace std::chrono_literals;

MultibotServer::MultibotServer()
: Node("server")
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
}

MultibotServer::~MultibotServer()
{
    RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
}
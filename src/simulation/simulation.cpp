#include "simulation/simulation.hpp"

using namespace Simulation;
using namespace std::chrono_literals;

MultibotSim::MultibotSim()
: Node("simulation")
{
    RCLCPP_INFO(this->get_logger(), "MultibotSim has been initialized");
}

MultibotSim::~MultibotSim()
{
    RCLCPP_INFO(this->get_logger(), "MultibotSim has been terminated");
}
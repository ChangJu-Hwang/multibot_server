#include <iostream>
#include <memory>

#include "simulation/simulation.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto simul = std::make_shared<Simulation::MultibotSim>();
    rclcpp::spin(simul);
    rclcpp::shutdown();

    return 0;
}
#include <iostream>
#include <memory>

#include "simulation/simulation.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto simul = std::make_shared<Simulation::MultibotSim>();
    simul->registration_request();

    rclcpp::spin(simul);

    rclcpp::shutdown();
    return 0;
}
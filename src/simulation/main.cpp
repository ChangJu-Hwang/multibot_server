#include <iostream>
#include <memory>

#include "simulation/simulation.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto simul = std::make_shared<Simulation::MultibotSim>();

    simul->register_robots();
    rclcpp::spin(simul);
    // rclcpp::WallRate loop_rate(10ms);
    // while(rclcpp::ok())
    // {
    //     rclcpp::spin_some(simul);
    //     simul->set_odomSubscribers();

    //     loop_rate.sleep();
    // }

    rclcpp::shutdown();
    return 0;
}
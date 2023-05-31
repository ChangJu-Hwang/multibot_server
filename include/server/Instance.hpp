#pragma once

#include "server/MAPF_Util.hpp"

using namespace MAPF_Util;

namespace Instance
{
    namespace AgentInstance
    {
        struct Agent
        {
            std::string name_;

            double size_;
            double wheel_radius_;
            double wheel_seperation_;

            double max_linVel_, max_linAcc_;
            double max_angVel_, max_angAcc_;

            Position::Pose start_;
            Position::Pose goal_;
            Position::Pose pose_;

            friend std::ostream &operator<<(std::ostream &_os, const Agent &_agent)
            {
                _os << "Agent Info"             << std::endl;
                _os << "- Name: "               << _agent.name_             << std::endl;
                _os << "- Size: "               << _agent.size_             << std::endl;
                _os << "- Wheel Radius: "       << _agent.wheel_radius_     << std::endl;
                _os << "- Wheel Seperation: "   << _agent.wheel_seperation_ << std::endl;
                _os << "- Start: "              << _agent.start_            << std::endl;
                _os << "- Goal: "               << _agent.goal_             << std::endl;
                _os << "- Current Pose: "       << _agent.pose_             << std::endl;
                
                _os << "- Maximum linear velocity     : "   << _agent.max_linVel_   << std::endl;
                _os << "- Maximum linear acceleration : "   << _agent.max_linAcc_   << std::endl;
                _os << "- Maximum angular acceleration: "   << _agent.max_angVel_   << std::endl;
                _os << "- Maximum angular acceleration: "   << _agent.max_angAcc_   << std::endl;

                return _os;
            }

            Agent() {} 
        }; // struct Agent
    } // namespace AgnetInstance

    namespace MapInstance
    {
        struct Cell
        {
            Position::Index _idx;
            
        }; // struct Cell
    } // namespace MapInstance
} // namespace Instance
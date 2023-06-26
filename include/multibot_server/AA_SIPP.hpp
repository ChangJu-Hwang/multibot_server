#pragma once

#include "multibot_util/Interface/Observer_Interface.hpp"
#include "multibot_util/MAPF_Util.hpp"

#include "multibot_server/Instance_Manager.hpp"
#include "multibot_server/AA_SIPP_Motion.hpp"

using namespace MAPF_Util;
using namespace Instance;

namespace Low_Level_Engine
{
    namespace AA_SIPP
    {
        class Planner : public Observer::ObserverInterface<InstanceMsg>
        {
        public:
            std::pair<Path::SinglePath, bool> search(
                const std::string &_agentName,
                const double _timeLimit = 10,
                const std::vector<std::string> &_higher_agents = std::vector<std::string>(),
                const Path::PathSet &_pathSet = Path::PathSet());

        private:
            std::pair<Path::SinglePath, bool> find_partial_path(
                const std::string &_agentName,
                const std::list<Time::TimeInterval> &_starts,
                const std::list<Time::TimeInterval> &_goals,
                const double _timeLimit);

        public:
            void update(const InstanceMsg &_msg)
            {
                agents_ = _msg.first;
                map_ = _msg.second;
            }

        private:
            std::unordered_map<std::string, AgentInstance::Agent> agents_;
            MapInstance::BinaryOccupancyMap map_;

            std::shared_ptr<Motion> motion_manager_ = std::make_shared<Motion>();

        public:
            Planner(std::shared_ptr<Instance_Manager> _instance_manager);
            ~Planner() {}
        }; // class Planner
    }      // namespace AA_SIPP
} // namespace Low_Level_Engine
#pragma once

#include "multibot_util/MAPF_Util.hpp"
#include "multibot_server/AA_SIPP.hpp"
#include "multibot_server/Instance_Manager.hpp"

using namespace Low_Level_Engine;
using namespace Instance;

namespace High_Level_Engine
{
    namespace CPBS
    {
        class Solver : public Observer::ObserverInterface<InstanceMsg>
        {
        public:
            void solve();
        
        private:
            std::pair<Position::Coordinates, Position::Coordinates> restrict_searchSpace();

        public:
            void update(const InstanceMsg &_msg)
            {
                agents_ = _msg.first;
            }
        
        private:
            std::unordered_map<std::string, AgentInstance::Agent> agents_;

            std::shared_ptr<AA_SIPP::Planner> planner_;

        public:
            Solver(std::shared_ptr<Instance_Manager> _instance_manager);
            ~Solver() {}
        }; // class Solver
    } // namespace CPBS
} // namespace High_Level_Engine
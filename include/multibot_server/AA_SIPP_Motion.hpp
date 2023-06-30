#pragma once

#include "multibot_util/Interface/Observer_Interface.hpp"

#include "multibot_server/Instance_Manager.hpp"
#include "multibot_server/AA_SIPP_Map_Utility.hpp"

using namespace MAPF_Util;
using namespace Instance;

namespace Low_Level_Engine
{
    namespace AA_SIPP
    {
        class Motion : public Observer::ObserverInterface<InstanceMsg>
        {
        public:
            const Time::TimePoint getMoveTime(
                const std::string &_agentName,
                const Position::Pose &_from, const Position::Pose &_to) const;
            const Time::TimePoint getPartialMoveTime(
                const std::string &_agentName, const Position::Pose &_target,
                const Position::Pose &_from, const Position::Pose &_to) const;
            const Time::TimePoint getPartialMoveTime(
                const std::string &_agentName, const Position::Index &_target,
                const Position::Pose &_from, const Position::Pose &_to) const;
            const Position::Pose getPosition(
                const std::string &_agentName, const Time::TimePoint &_key, const Time::TimePoint &_start_time,
                const Position::Pose &_from, const Position::Pose &_to);

        private:
            const Time::TimePoint MoveTimeComputer(
                const double _s,
                const double _max_s, const double _max_v, const double _max_a) const;
            double DisplacementComputer(
                const Time::TimePoint &_key,
                const double _max_s, const double _max_v, const double _max_a) const;

        public:
            void update(const InstanceMsg &_msg)
            {
                agents_ = _msg.first;
            }

        private:
            std::unordered_map<std::string, AgentInstance::Agent> agents_;

            std::shared_ptr<Map_Utility> map_utility_ = std::make_shared<Map_Utility>();

        public:
            Motion(std::shared_ptr<Map_Utility> _map_utility)
                : map_utility_(_map_utility) {}
            ~Motion() {}

        }; // class Motion
    } // namespace AA_SIPP
} // namespace Low_Level_Engine
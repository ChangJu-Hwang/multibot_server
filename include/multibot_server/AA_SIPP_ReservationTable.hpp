#pragma once

#include "multibot_util/Interface/Observer_Interface.hpp"

#include "multibot_server/Instance_Manager.hpp"
#include "multibot_server/AA_SIPP_Map_Utility.hpp"
#include "multibot_server/AA_SIPP_Motion.hpp"
#include "multibot_server/AA_SIPP_Conflict_Checker.hpp"

using namespace MAPF_Util;
using namespace Instance;

namespace Low_Level_Engine
{
    namespace AA_SIPP
    {
        class ReservationTable : public Observer::ObserverInterface<InstanceMsg>
        {
        public:
            void init(double _inflation_radius);
            bool applySinglePath(
                const std::string &_agentName, const Path::SinglePath &_singlePath);
            bool isValidMove(
                const std::string &_agentName, const Time::TimePoint &_departure_time,
                const Position::Pose &_from, const Position::Pose &_to);
            const std::list<Time::TimeInterval> getSafeIntervals(const Position::Index &_index) const;
            const std::list<Time::TimeInterval> getSafeIntervals(const Position::Pose &_pose) const;
        
        private:
            bool addCollisionInterval(
                const Position::Index &_index,
                const Time::TimePoint &_startTime, const Time::TimePoint &_endTime);

        public:
            void update(const InstanceMsg &_msg)
            {
                for (const auto &agent : _msg.first)
                {
                    agentSizeDB_.insert(std::make_pair(
                        agent.second.name_, agent.second.size_));
                }

                map_ = _msg.second;
            }

        private:
            std::unordered_map<std::string, double> agentSizeDB_;
            MapInstance::BinaryOccupancyMap map_;

            std::vector<std::vector<Time::TimeLine>> reservationTable_;

            std::shared_ptr<Map_Utility> map_utility_;
            std::shared_ptr<Motion> motion_manager_;
            std::shared_ptr<ConflictChecker> conflict_checker_;

        public:
            ReservationTable(
                std::shared_ptr<Map_Utility> _map_utility,
                std::shared_ptr<Motion> _motion_manager,
                std::shared_ptr<ConflictChecker> _conflict_checker)
                : map_utility_(_map_utility),
                  motion_manager_(_motion_manager),
                  conflict_checker_(_conflict_checker) {}
            ~ReservationTable() {}
        }; // class ReservationTable
    } // namespace AA_SIPP
} // namespace Low_Level_Engine
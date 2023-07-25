#pragma once

#include <set>

#include "multibot_util/Interface/Observer_Interface.hpp"
#include "multibot_util/MAPF_Util.hpp"

#include "multibot_server/Instance_Manager.hpp"
#include "multibot_server/AA_SIPP_Node.hpp"
#include "multibot_server/AA_SIPP_Map_Utility.hpp"
#include "multibot_server/AA_SIPP_Motion.hpp"
#include "multibot_server/AA_SIPP_Conflict_Checker.hpp"
#include "multibot_server/AA_SIPP_ReservationTable.hpp"

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
                const std::vector<std::string> &_higher_agents = std::vector<std::string>(),
                const Path::PathSet &_pathSet = Path::PathSet(),
                const std::pair<Position::Coordinates, Position::Coordinates> &_searchSpace
                 = std::make_pair(
                    Position::Coordinates(-1 * std::numeric_limits<double>::infinity(), -1 * std::numeric_limits<double>::infinity()),
                    Position::Coordinates(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity())),
                const double _timeLimit = 10);

        private:
            std::pair<Path::SinglePath, bool> find_partial_path(
                const std::string &_agentName,
                const std::list<Time::TimeInterval> &_starts,
                const std::list<Time::TimeInterval> &_goals,
                const double _timeLimit);
            void generateRoot(
                const Position::Pose &_startPose,
                const std::list<Time::TimeInterval> &_starts);
            std::vector<AA_SIPP::Node> getSuccessors(
                const std::string &_agentName,
                const Position::Index &_index, const AA_SIPP::Node &_parentNode);
            Path::SinglePath constructSinglePath(
                const std::string &_agentName, const AA_SIPP::Node &_goalNode);

            double computeTotalDelays(
                const std::string &_agentName,
                const Position::Pose &_startPose, const Position::Pose &_goalPose,
                const Time::TimePoint &_departure_time, const Time::TimePoint &_max_delay_limit);

        public:
            void update(const InstanceMsg &_msg)
            {
                agents_ = _msg.first;
            }

        private:
            std::unordered_map<std::string, AgentInstance::Agent> agents_;

            std::vector<Path::SinglePath> higher_paths_;

            std::set<AA_SIPP::Node, AA_SIPP::Compare> open_;
            std::unordered_map<AA_SIPP::Node, AA_SIPP::Node, AA_SIPP::Node_HashFunc> close_;

            std::shared_ptr<Map_Utility> map_utility_ = std::make_shared<Map_Utility>();
            std::shared_ptr<Motion> motion_manager_ = std::make_shared<Motion>(map_utility_);
            std::shared_ptr<ConflictChecker> conflict_checker_ = std::make_shared<ConflictChecker>(motion_manager_);
            std::shared_ptr<ReservationTable> reservation_table_ = std::make_shared<ReservationTable>(
                map_utility_, motion_manager_, conflict_checker_);

        public:
            Planner(std::shared_ptr<Instance_Manager> &_instance_manager);
            ~Planner() {}
        }; // class Planner

    } // namespace AA_SIPP
} // namespace Low_Level_Engine
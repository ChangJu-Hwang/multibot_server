#pragma once

#include "multibot_util/Interface/Observer_Interface.hpp"

#include "multibot_server/AA_SIPP_Motion.hpp"

namespace Low_Level_Engine
{
    namespace AA_SIPP
    {
        class ConflictChecker : public Observer::ObserverInterface<InstanceMsg>
        {
        private:
            using PartialPath = std::pair<Path::SinglePath::Node, Path::SinglePath::Node>;
            enum SEARCH{LEFT, RIGHT};
            enum DIRECTION{FORWARD, REVERSE};

        public:
            Time::TimeInterval checkConflict(
                const Path::SinglePath &_higherPath, const Path::SinglePath &_lowerPath);

        private:
            Time::TimeInterval checkPartialConflict(
                const std::string &_higherName,  std::vector<PartialPath>::const_iterator _higher_PartialPath,
                const std::string &_lowerName, std::vector<PartialPath>::const_iterator _lower_PartialPath);
            Time::TimeInterval findCriticalInterval(
                const std::string &_higherName,  const PartialPath &_higher_PartialPath,
                const std::string &_lowerName, const PartialPath &_lower_PartialPath,
                const Time::TimePoint &_lower_bound, const Time::TimePoint &_upper_bound);
            Time::TimePoint binarySearch(
                bool _mode,
                const std::string &_higherName,  const PartialPath &_higher_PartialPath,
                const std::string &_lowerName, const PartialPath &_lower_PartialPath,
                const Time::TimePoint &_lower_bound, const Time::TimePoint &_upper_bound);
            Time::TimeInterval getCollisionInterval(
                const Time::TimePoint _time, const double _safe_distance,
                const Path::SinglePath &_higherPath, std::vector<PartialPath>::const_iterator _higher_PartialPath,
                const Position::Pose &_lower_pose);

        public:
            void update(const InstanceMsg &_msg)
            {
                for (const auto &agent : _msg.first)
                {
                    agentSizeDB_.insert(std::make_pair(
                        agent.second.name_, agent.second.size_));
                }
            }

        private:
            std::unordered_map<std::string, double> agentSizeDB_;
            static constexpr double time_precision_ = 1e-2;

            std::shared_ptr<Motion> motion_manager_;

        public:
            ConflictChecker(std::shared_ptr<Motion> &_motion_manager)
            : motion_manager_(_motion_manager) {}
            ~ConflictChecker() {}
        }; // class ConflictChecker
    } // namespace Low_Level_Search
} // namespace Low_Level_Search
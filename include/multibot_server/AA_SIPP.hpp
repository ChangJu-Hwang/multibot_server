#pragma once

#include "multibot_util/MAPF_Util.hpp"

using namespace MAPF_Util;

namespace Low_Level_Engine
{
    namespace AA_SIPP
    {
        struct Node
        {
        public:
            void updateHeuristic(
                const Position::Pose &_goal,
                const double _max_linVel);
            bool validationCheck() const;

        public:
            Node &operator=(const AA_SIPP::Node &_other);
            bool operator==(const AA_SIPP::Node &_other) const;
            bool operator!=(const AA_SIPP::Node &_other) const;

            friend std::ostream &operator<<(std::ostream &_os, const AA_SIPP::Node &_node)
            {
                _os << _node.pose_ << ": "
                    << "Arrive at " << _node.arrival_time_.count() << "s, "
                    << "Depart at " << _node.departure_time_.count() << "s";
                
                return _os;
            }
        
        public:
            Position::Index idx_;
            Position::Pose pose_;
            
            Time::TimeInterval safe_interval_;
            Time::TimePoint arrival_time_;
            Time::TimePoint departure_time_;

            double gVal_;
            double hVal_;

            const Node* parent_;
        
        public:
            Node(const Node &_other);
            Node(
                Position::Index _idx = Position::Index(),
                Position::Pose _pose = Position::Pose(),
                Time::TimeInterval _safe_interval = Time::TimeInterval(),
                Time::TimePoint _arrival_time = Time::TimePoint::zero(),
                Time::TimePoint _departure_time = Time::TimePoint::zero(),
                double _gVal = std::numeric_limits<double>::quiet_NaN(),
                double _hVal = std::numeric_limits<double>::quiet_NaN(),
                Node *_parent = nullptr)
                : idx_(_idx), pose_(_pose), safe_interval_(_safe_interval),
                  arrival_time_(_arrival_time), departure_time_(_departure_time),
                  gVal_(_gVal), hVal_(_hVal), parent_(_parent) {}
        }; // struct Node

        class Planner
        {
        public:
            void search();

        public:
            Planner() {}
            ~Planner() {}
        }; // class Planner
    } // namespace AA_SIPP
} // namespace Low_Level_Engine
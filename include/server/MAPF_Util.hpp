#pragma once

#include <iostream>
#include <chrono>
#include <cmath>
#include <list>
#include <map>

#include <geometry_msgs/msg/pose2_d.hpp>

namespace MAPF_Util
{
    namespace Position
    {
        enum CARTESIAN{x, y, theta}; // enum CARTESIAN

        struct Index
        {
            int x_, y_;

            Index &operator=(const Index &_other)
            {
                x_  = _other.x_;
                y_  = _other.y_;

                return *this;
            }

            bool operator==(const Index &_other) const
            {
                return (x_ == _other.x_ and y_ == _other.y_);
            }

            bool operator!=(const Index &_other) const
            {
                return not(*this == _other);
            }

            friend std::ostream &operator<<(std::ostream &_os, const Index &_index)
            {
                return _os << "[" << _index.x_ << "]"
                           << "[" << _index.y_ << "]";
            }

            Index(const Index &_other)
            {
                x_  = _other.x_;
                y_  = _other.y_;
            }

            Index(int _x = -1, int _y = -1)
                : x_(_x), y_(_y) {}
        }; // struct Index

        struct Coordinates
        {
            double x_, y_;

            Coordinates &operator=(const Coordinates &_other);
            Coordinates &operator=(const std::vector<double> &_other);
            Coordinates &operator=(const std::pair<double, double> &_other);

            bool operator==(const Coordinates &_other) const;
            bool operator!=(const Coordinates &_other) const;

            const Coordinates operator+(const Coordinates &_other) const;
            const Coordinates operator-(const Coordinates &_other) const;
            const Coordinates operator*(const double &_coefficient) const;
            const double operator*(const Coordinates &_other) const;
            const Coordinates operator/(const double &_divider) const;

            const double norm();

            friend Coordinates operator-(const Coordinates &_coord)
            {
                return _coord * -1;
            }

            friend Coordinates operator*(const double &_coefficient, const Coordinates &_coord)
            {
                return _coord * _coefficient;
            }

            friend std::ostream &operator<<(std::ostream &_os, const Coordinates &_coordinates)
            {
                return _os << "("   << _coordinates.x_
                           << ", "  << _coordinates.y_ << ")";
            }

            Coordinates(const Coordinates &_other)
            {
                x_  = _other.x_;
                y_  = _other.y_;
            }

            Coordinates(double _x = std::numeric_limits<double>::quiet_NaN(), double _y = std::numeric_limits<double>::quiet_NaN())
                : x_(_x), y_(_y) {};
        }; // struct Coordinates

        struct Pose
        {
            geometry_msgs::msg::Pose2D component_;

            Pose &operator=(const Pose &_other)
            {
                component_ = _other.component_;
                
                return *this;
            }

            bool operator==(const Pose &_other) const
            {
                return component_ == _other.component_;
            }

            bool operator!=(const Pose &_other) const
            {
                return component_ != _other.component_;
            }

            friend std::ostream &operator<<(std::ostream &_os, const Pose &_pose)
            {
                _os << "("  << _pose.component_.x << "m"
                    << ", " << _pose.component_.y << "m"
                    << ", " << _pose.component_.theta << "rad"
                    << ")";
                
                return _os;
            }

            Pose(geometry_msgs::msg::Pose2D _component = geometry_msgs::msg::Pose2D())
                : component_(_component) {}

            Pose(double _x, double _y, double _theta)
            {
                component_.x        = _x;
                component_.y        = _y;
                component_.theta    = _theta;
            }
        }; // struct Pose

        double getDistance(const Coordinates &_first, const Coordinates &_second);
        double crossProduct(const Coordinates &_first, const Coordinates &_second);
        double getDistance(const Pose &_first, const Pose &_second);
        double getAngleDiff(const Pose &_first, const Pose &_second);
    } // namespace Position

    namespace Time
    {
        typedef std::chrono::duration<double> TimePoint;

        struct TimeInterval
        {
            TimePoint startTime_;
            TimePoint endTime_;
            bool is_safe_;

            friend std::ostream &operator<<(std::ostream &_os, const TimeInterval &_timeInterval)
            {
                return _os << "["  << _timeInterval.startTime_.count() << "s"
                           << ", " << _timeInterval.endTime_.count() << "s"
                           << ")";
            }

            TimeInterval &operator=(const TimeInterval &_other)
            {
                startTime_  = _other.startTime_;
                endTime_    = _other.endTime_;
                is_safe_    = _other.is_safe_;

                return *this;
            }

            bool operator==(const TimeInterval &_other) const
            {
                if (std::fabs(startTime_.count() - _other.startTime_.count()) > 1e-8)
                    return false;
                
                if (endTime_ == TimePoint::max() and _other.endTime_ == TimePoint::max())
                    return true;
                else if (std::fabs(endTime_.count() - _other.endTime_.count()) > 1e-8)
                    return false;
                else    
                    return true;
            }

            bool operator!=(const TimeInterval &_other) const
            {
                return not(*this == _other);
            }

            TimeInterval(TimePoint _startTime = TimePoint::max(), TimePoint _endTime = TimePoint::max(), bool _is_safe = false)
                : startTime_(_startTime), endTime_(_endTime), is_safe_(_is_safe) {}
        }; // struct TimeInterval

        struct TimeLine
        {
            Position::Index idx_;
            std::list<TimeInterval> interval_list_;
            bool occupied_;

            friend std::ostream &operator<<(std::ostream &_os, const TimeLine &_timeLine)
            {
                _os << "TimeLine" << _timeLine.idx_ << std::endl;
                for (const auto &TimeInterval : _timeLine.interval_list_)
                {
                    if (TimeInterval.is_safe_)
                        _os << "o Safe Interval     : ";
                    else
                        _os << "x Collision Interval: ";
                    _os << TimeInterval << std::endl;
                }

                return _os;
            }

            TimeLine &operator=(const TimeLine &_other)
            {
                idx_            = _other.idx_;
                interval_list_  = _other.interval_list_;
                occupied_       = _other.occupied_;

                return *this;
            }

            TimeLine(Position::Index _idx = Position::Index(), bool _occupied = false)
                : idx_(_idx), occupied_(_occupied)
            {
                interval_list_.clear();
            }
        }; // struct TimeLine
    } // namespace Time

    namespace Path
    {
        struct SinglePath
        {
            struct Node
            {
                Position::Pose pose_;
                Time::TimePoint arrival_time_;
                Time::TimePoint departure_time_;

                Node &operator=(const Node &_other)
                {
                    pose_           = _other.pose_;
                    arrival_time_   = _other.arrival_time_;
                    departure_time_ = _other.departure_time_;

                    return *this;
                }
            }; // struct Node

            std::string agentName_;
            std::vector<std::pair<SinglePath::Node, SinglePath::Node>> nodes_;
            double cost_;

            friend std::ostream &operator<<(std::ostream &_os, const SinglePath &_singlePath)
            {
                _os.precision(4);
                _os << "[" << _singlePath.agentName_ << "]"
                    << "Cost: " << _singlePath.cost_ << "s" << std::endl;

                for(const auto& nodePair : _singlePath.nodes_)
                {
                    _os << "["  << _singlePath.agentName_ << "]"
                        << "["  << nodePair.first.departure_time_.count() << "s"
                        << ", " << nodePair.second.arrival_time_.count()  << "s" << ")"
                        << ": " << nodePair.first.pose_ << " -> " << nodePair.second.pose_
                        << std::endl;
                }

                return _os;                    
            }

            SinglePath() {}
        }; // struct SinglePath

        typedef std::map<std::string, SinglePath> PathSet;
    } // namespace Path
} // namespace MAPF_Util
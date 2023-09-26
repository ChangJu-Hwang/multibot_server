#include "multibot_server/AA_SIPP_Conflict_Checker.hpp"

using namespace Low_Level_Engine;

double AA_SIPP::ConflictChecker::getDelayTime(
    const Traj::SingleTraj &_higherTraj, const Traj::SingleTraj &_lowerTraj)
{
    auto higher_PartialTraj = _higherTraj.nodes_.begin();
    auto lower_PartialTraj = _lowerTraj.nodes_.begin();

    while (higher_PartialTraj != _higherTraj.nodes_.end() or lower_PartialTraj != _lowerTraj.nodes_.end())
    {
        if (Position::getDistance(higher_PartialTraj->first.pose_, higher_PartialTraj->second.pose_) +
                Position::getDistance(lower_PartialTraj->first.pose_, lower_PartialTraj->second.pose_) >
            Position::getDistance(higher_PartialTraj->first.pose_, lower_PartialTraj->first.pose_) -
                (agentSizeDB_[_higherTraj.agentName_] + agentSizeDB_[_lowerTraj.agentName_] + 1e-8))
        {
            bool isConflict = false;
            double max_delay = -1e-8;

            while (true)
            {
                if (checkPartialConflict(
                        _higherTraj.agentName_, higher_PartialTraj,
                        _lowerTraj.agentName_, lower_PartialTraj,
                        max_delay) == false)
                {
                    break;
                }

                isConflict = true;
                ++higher_PartialTraj;

                if (higher_PartialTraj == _higherTraj.nodes_.end())
                    return (std::numeric_limits<double>::max() - 1e-8);

                max_delay = getDelayScope(
                    _higherTraj.agentName_, higher_PartialTraj,
                    _lowerTraj.agentName_, lower_PartialTraj,
                    agentSizeDB_[_higherTraj.agentName_] + agentSizeDB_[_lowerTraj.agentName_]);
            }

            if (isConflict)
            {
                if (max_delay > std::numeric_limits<double>::max() - 1e-3)
                    return std::numeric_limits<double>::max();

                double delay = max_delay / 2;
                while (checkPartialConflict(
                    _higherTraj.agentName_, higher_PartialTraj,
                    _lowerTraj.agentName_, lower_PartialTraj,
                    delay))
                {
                    // Suboptimal Delay Time
                    delay = (delay + max_delay) / 2;

                    if (std::fabs(max_delay - delay) < 1e-3)
                        break;
                }

                return delay;
            }
        }

        if (higher_PartialTraj == std::prev(_higherTraj.nodes_.end(), 1) and
            lower_PartialTraj == std::prev(_lowerTraj.nodes_.end(), 1))
        {
            ++higher_PartialTraj;
            ++lower_PartialTraj;
        }
        else if (higher_PartialTraj == std::prev(_higherTraj.nodes_.end(), 1))
            ++lower_PartialTraj;
        else if (lower_PartialTraj == std::prev(_lowerTraj.nodes_.end(), 1))
            ++higher_PartialTraj;
        else if (std::fabs((higher_PartialTraj->second.arrival_time_ - lower_PartialTraj->second.arrival_time_).count()) < 1e-8)
        {
            ++higher_PartialTraj;
            ++lower_PartialTraj;
        }
        else if (lower_PartialTraj->second.arrival_time_ > higher_PartialTraj->second.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon())
            ++higher_PartialTraj;
        else if (higher_PartialTraj->second.arrival_time_ > lower_PartialTraj->second.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon())
            ++lower_PartialTraj;
        else
        {
            ++higher_PartialTraj;
            ++lower_PartialTraj;
        }
    }

    return 0;
}

std::pair<Position::Index, Position::Index> AA_SIPP::ConflictChecker::getConflictScope(
    const Position::Index &_target,
    const std::vector<Position::Index> &_indexGroup,
    const double _safe_distance)
{
    Position::Coordinates target = map_.mapData_[_target.x_][_target.y_].coord_;

    Position::Index conflict_start, conflict_end;
    bool conflictFlag = false, resultFlag = false;

    for (const auto &index : _indexGroup)
    {
        Position::Coordinates routeComponent = map_.mapData_[index.x_][index.y_].coord_;
        double distance = Position::getDistance(target, routeComponent);

        if (not(conflictFlag) and _safe_distance + std::sqrt(2) * map_.property_.resolution_ + 1e-8 > distance)
        {
            conflictFlag = true, resultFlag = true;
            conflict_start = index;
        }

        if (conflictFlag and distance + 1e-8 > _safe_distance + std::sqrt(2) * map_.property_.resolution_)
        {
            conflictFlag = false;
            auto index_iter = std::find(_indexGroup.begin(), _indexGroup.end(), index);
            conflict_end = *std::prev(index_iter, 1);
            break;
        }
    }

    if (not(resultFlag))
        return std::pair(Position::Index(), Position::Index());

    if (conflictFlag)
        conflict_end = _indexGroup.back();

    return std::pair(conflict_start, conflict_end);
}

bool AA_SIPP::ConflictChecker::checkPartialConflict(
    const std::string &_higherName, std::vector<PartialTraj>::const_iterator _higher_PartialTraj,
    const std::string &_lowerName, std::vector<PartialTraj>::const_iterator _lower_PartialTraj,
    double _delay)
{
    Time::TimePoint lower_bound = std::max(_higher_PartialTraj->first.departure_time_, _lower_PartialTraj->first.departure_time_ + Time::TimePoint(_delay));
    Time::TimePoint upper_bound = std::min(_higher_PartialTraj->second.arrival_time_, _lower_PartialTraj->second.arrival_time_ + Time::TimePoint(_delay));

    return conflictSearch(
        _higherName, *_higher_PartialTraj,
        _lowerName, *_lower_PartialTraj,
        lower_bound, upper_bound,
        _delay);
}

double AA_SIPP::ConflictChecker::getDelayScope(
    const std::string &_higherName, std::vector<PartialTraj>::const_iterator _higher_PartialTraj,
    const std::string &_lowerName, std::vector<PartialTraj>::const_iterator _lower_PartialTraj,
    double _safe_distance)
{
    if (_lower_PartialTraj->first.departure_time_.count() > std::numeric_limits<double>::max() - 1e-3)
        return std::numeric_limits<double>::max() - 1e-8;

    Position::Coordinates higher_start(
        _higher_PartialTraj->first.pose_.component_.x,
        _higher_PartialTraj->first.pose_.component_.y);

    Position::Coordinates higher_end(
        _higher_PartialTraj->second.pose_.component_.x,
        _higher_PartialTraj->second.pose_.component_.y);

    Position::Coordinates lower_start(
        _lower_PartialTraj->first.pose_.component_.x,
        _lower_PartialTraj->first.pose_.component_.y);

    Position::Coordinates lower_end(
        _lower_PartialTraj->second.pose_.component_.x,
        _lower_PartialTraj->second.pose_.component_.y);

    Position::Coordinates higher_vector = higher_end - higher_start;
    Position::Coordinates lower_vector = lower_end - lower_start;

    Position::Coordinates higher_unit_vector = higher_vector, lower_unit_vector = lower_vector;
    if (higher_vector.norm() < 1e-8 or lower_unit_vector.norm() < 1e-8)
        return 0;

    higher_unit_vector = higher_vector / higher_vector.norm();
    lower_unit_vector = lower_vector / lower_vector.norm();

    double sinTheta = Position::crossProduct(higher_unit_vector, lower_unit_vector);

    Position::Coordinates higher_p, lower_p;
    if (std::fabs(sinTheta) > 1e-8)
    {
        Position::Coordinates t = lower_start - higher_start;

        double higher_height = Position::crossProduct(t, lower_unit_vector);
        double higher_t = higher_height / sinTheta;
        higher_p = higher_start + higher_t * higher_unit_vector;

        if (higher_t < 1e-8)
            higher_p = higher_start;
        else if (higher_t > higher_vector.norm() + 1e-8)
            higher_p = higher_end;

        double lower_height = Position::crossProduct(t, higher_unit_vector);
        double lower_t = lower_height / sinTheta;
        lower_p = lower_start + lower_t * lower_unit_vector;

        if (lower_t < 1e-8)
            lower_p = lower_start;
        else if (lower_t > lower_vector.norm() + 1e-8)
            lower_p = lower_end;

        if (higher_t < 1e-8 or higher_t > higher_vector.norm() + 1e-8)
        {
            double dotValue = lower_unit_vector * (higher_p - lower_start);
            if (dotValue < 1e-8)
                dotValue = 0;
            else if (dotValue > lower_vector.norm() + 1e-8)
                dotValue = lower_vector.norm();

            lower_p = lower_start + dotValue * lower_unit_vector;
        }

        if (lower_t < 1e-8 or lower_t > lower_vector.norm() + 1e-8)
        {
            double dotValue = higher_unit_vector * (lower_p - higher_start);
            if (dotValue < 1e-8)
                dotValue = 0;
            else if (dotValue > higher_vector.norm() + 1e-8)
                dotValue = higher_vector.norm();

            higher_p = higher_start + dotValue * higher_unit_vector;
        }
    }
    else
    {
        double entire_scope = _higher_PartialTraj->second.arrival_time_.count() - _higher_PartialTraj->first.departure_time_.count();
        return entire_scope;
    }

    double higher_height = std::fabs(Position::crossProduct(higher_p - lower_start, lower_unit_vector));
    Position::Coordinates higher_escape_coord = higher_p + (_safe_distance - higher_height) / std::fabs(sinTheta) * higher_unit_vector;

    if (Position::getDistance(higher_escape_coord, higher_end) > higher_vector.norm() + 1e-8)
        higher_escape_coord = higher_start;
    else if (Position::getDistance(higher_escape_coord, higher_start) > higher_vector.norm() + 1e-8)
        higher_escape_coord = higher_end;

    double lower_height = std::fabs(Position::crossProduct(lower_p - higher_start, higher_unit_vector));
    Position::Coordinates lower_enter_coord = lower_p - (_safe_distance - lower_height) / std::fabs(sinTheta) * lower_unit_vector;
    if (Position::getDistance(lower_enter_coord, lower_end) > lower_vector.norm() + 1e-8)
        lower_enter_coord = lower_start;
    else if (Position::getDistance(lower_enter_coord, lower_start) > lower_vector.norm() + 1e-8)
        lower_enter_coord = lower_end;

    Time::TimePoint reference_timePoint = _lower_PartialTraj->first.departure_time_ + motion_manager_->getPartialMoveTime(
                                                                                          _lowerName, Position::Pose(lower_enter_coord.x_, lower_enter_coord.y_, _lower_PartialTraj->second.pose_.component_.theta),
                                                                                          _lower_PartialTraj->first.pose_, _lower_PartialTraj->second.pose_);

    Time::TimePoint escape_timePoint = _higher_PartialTraj->first.departure_time_ + motion_manager_->getPartialMoveTime(
                                                                                        _higherName, Position::Pose(higher_escape_coord.x_, higher_escape_coord.y_, _higher_PartialTraj->second.pose_.component_.theta),
                                                                                        _higher_PartialTraj->first.pose_, _higher_PartialTraj->second.pose_);

    return (escape_timePoint - reference_timePoint).count();
}

bool AA_SIPP::ConflictChecker::conflictSearch(
    const std::string &_higherName, const PartialTraj &_higher_PartialTraj,
    const std::string &_lowerName, const PartialTraj &_lower_PartialTraj,
    const Time::TimePoint &_lower_bound, const Time::TimePoint &_upper_bound,
    double _delay)
{
    if (std::fabs((_upper_bound - _lower_bound).count()) < time_precision_)
        return false;

    double key = (_lower_bound + _upper_bound).count() / 2;

    Position::Pose higher_pos = motion_manager_->getPosition(
        _higherName, Time::TimePoint(key), _higher_PartialTraj.first.departure_time_,
        _higher_PartialTraj.first.pose_, _higher_PartialTraj.second.pose_);

    Position::Pose lower_pos = motion_manager_->getPosition(
        _lowerName, Time::TimePoint(key - _delay), _lower_PartialTraj.first.departure_time_,
        _lower_PartialTraj.first.pose_, _lower_PartialTraj.second.pose_);

    double relative_distance = Position::getDistance(higher_pos, lower_pos);

    if (relative_distance > agentSizeDB_[_higherName] + agentSizeDB_[_lowerName] + 1e-8)
    {
        Position::Pose next_higher_pos = motion_manager_->getPosition(
            _higherName, Time::TimePoint(key + time_precision_), _higher_PartialTraj.first.departure_time_,
            _higher_PartialTraj.first.pose_, _higher_PartialTraj.second.pose_);

        Position::Pose next_lower_pos = motion_manager_->getPosition(
            _lowerName, Time::TimePoint(key + time_precision_ - _delay), _lower_PartialTraj.first.departure_time_,
            _lower_PartialTraj.first.pose_, _lower_PartialTraj.second.pose_);

        double next_relative_distance = Position::getDistance(next_higher_pos, next_lower_pos);

        Time::TimePoint new_lower_bound = _lower_bound;
        Time::TimePoint new_upper_bound = _upper_bound;

        if (next_relative_distance - relative_distance > 1e-8)
            new_upper_bound = Time::TimePoint(key);
        else
            new_lower_bound = Time::TimePoint(key);

        return conflictSearch(
            _higherName, _higher_PartialTraj,
            _lowerName, _lower_PartialTraj,
            new_lower_bound, new_upper_bound,
            _delay);
    }
    else
        return true;
}
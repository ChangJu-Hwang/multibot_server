#include "multibot_server/AA_SIPP_Conflict_Checker.hpp"

using namespace Low_Level_Engine;

double AA_SIPP::ConflictChecker::getDelayTime(
    const Path::SinglePath &_higherPath, const Path::SinglePath &_lowerPath)
{
    auto higher_PartialPath = _higherPath.nodes_.begin();
    auto lower_PartialPath = _lowerPath.nodes_.begin();

    while (higher_PartialPath != _higherPath.nodes_.end() or lower_PartialPath != _lowerPath.nodes_.end())
    {
        if (Position::getDistance(higher_PartialPath->first.pose_, higher_PartialPath->second.pose_) +
                Position::getDistance(lower_PartialPath->first.pose_, lower_PartialPath->second.pose_) >
            Position::getDistance(higher_PartialPath->first.pose_, lower_PartialPath->first.pose_) -
                (agentSizeDB_[_higherPath.agentName_] + agentSizeDB_[_lowerPath.agentName_] + 1e-8))
        {
            bool isConflict = false;
            double max_delay = -1e-8;
            while (checkPartialConflict(
                _higherPath.agentName_, higher_PartialPath,
                _lowerPath.agentName_, lower_PartialPath,
                max_delay))
            {
                isConflict = true;
                if (max_delay > 1e-8)
                    ++higher_PartialPath;
                
                if (higher_PartialPath == _higherPath.nodes_.end())
                    return (std::numeric_limits<double>::max() - 1e-8);

                max_delay = getDelayScope(
                    _higherPath.agentName_, higher_PartialPath,
                    _lowerPath.agentName_, lower_PartialPath,
                    agentSizeDB_[_higherPath.agentName_] + agentSizeDB_[_lowerPath.agentName_]);
            }

            if (isConflict)
            {
                double delay = max_delay / 2;
                while (checkPartialConflict(
                    _higherPath.agentName_, higher_PartialPath,
                    _lowerPath.agentName_, lower_PartialPath,
                    delay))
                {
                    // Suboptimal Delay Time
                    delay = (delay + max_delay) / 2;
                }

                return delay;
            }
        }

        if (higher_PartialPath == std::prev(_higherPath.nodes_.end(), 1) and
            lower_PartialPath == std::prev(_lowerPath.nodes_.end(), 1))
        {
            ++higher_PartialPath;
            ++lower_PartialPath;
        }
        else if (higher_PartialPath == std::prev(_higherPath.nodes_.end(), 1))
            ++lower_PartialPath;
        else if (lower_PartialPath == std::prev(_lowerPath.nodes_.end(), 1))
            ++higher_PartialPath;
        else if (std::fabs((higher_PartialPath->second.arrival_time_ - lower_PartialPath->second.arrival_time_).count()) < 1e-8)
        {
            ++higher_PartialPath;
            ++lower_PartialPath;
        }
        else if (lower_PartialPath->second.arrival_time_ > higher_PartialPath->second.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon())
            ++higher_PartialPath;
        else if (higher_PartialPath->second.arrival_time_ > lower_PartialPath->second.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon())
            ++lower_PartialPath;
        else
        {
            ++higher_PartialPath;
            ++lower_PartialPath;
        }
    }

    return 0;
}

bool AA_SIPP::ConflictChecker::checkPartialConflict(
    const std::string &_higherName, std::vector<PartialPath>::const_iterator _higher_PartialPath,
    const std::string &_lowerName, std::vector<PartialPath>::const_iterator _lower_PartialPath,
    double _delay)
{
    Time::TimePoint lower_bound = std::max(_higher_PartialPath->first.departure_time_, _lower_PartialPath->first.departure_time_ + Time::TimePoint(_delay));
    Time::TimePoint upper_bound = std::min(_higher_PartialPath->second.arrival_time_, _lower_PartialPath->second.arrival_time_ + Time::TimePoint(_delay));

    return conflictSearch(
        _higherName, *_higher_PartialPath,
        _lowerName, *_lower_PartialPath,
        lower_bound, upper_bound,
        _delay);
}

double AA_SIPP::ConflictChecker::getDelayScope(
    const std::string &_higherName, std::vector<PartialPath>::const_iterator _higher_PartialPath,
    const std::string &_lowerName, std::vector<PartialPath>::const_iterator _lower_PartialPath,
    double _safe_distance)
{
    Position::Coordinates higher_start(
        _higher_PartialPath->first.pose_.component_.x,
        _higher_PartialPath->first.pose_.component_.y);

    Position::Coordinates higher_end(
        _higher_PartialPath->second.pose_.component_.x,
        _higher_PartialPath->second.pose_.component_.y);

    Position::Coordinates lower_start(
        _lower_PartialPath->first.pose_.component_.x,
        _lower_PartialPath->first.pose_.component_.y);

    Position::Coordinates lower_end(
        _lower_PartialPath->second.pose_.component_.x,
        _lower_PartialPath->second.pose_.component_.y);

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
        double entire_scope = _higher_PartialPath->second.arrival_time_.count() - _higher_PartialPath->first.departure_time_.count();
        return entire_scope;
    }

    double higher_height = std::fabs(Position::crossProduct(higher_p - lower_start, lower_unit_vector));
    Position::Coordinates higher_escape_coord = higher_p + (_safe_distance - higher_height) / std::fabs(sinTheta) * higher_unit_vector;
    if (Position::getDistance(higher_escape_coord, higher_start) > higher_vector.norm() + 1e-8)
        higher_escape_coord = higher_end;

    double lower_height = std::fabs(Position::crossProduct(lower_p - higher_start, higher_unit_vector));
    Position::Coordinates lower_enter_coord = lower_p - (_safe_distance - lower_height) / std::fabs(sinTheta) * lower_unit_vector;
    if (Position::getDistance(lower_enter_coord, lower_end) > lower_vector.norm() + 1e-8)
        lower_enter_coord = lower_start;

    Time::TimePoint reference_timePoint = _lower_PartialPath->first.departure_time_ + motion_manager_->getPartialMoveTime(
                                                                                          _lowerName, Position::Pose(lower_enter_coord.x_, lower_enter_coord.y_, _lower_PartialPath->second.pose_.component_.theta),
                                                                                          _lower_PartialPath->first.pose_, _lower_PartialPath->second.pose_);

    Time::TimePoint escape_timePoint = _higher_PartialPath->first.departure_time_ + motion_manager_->getPartialMoveTime(
                                                                                        _higherName, Position::Pose(higher_escape_coord.x_, higher_escape_coord.y_, _higher_PartialPath->second.pose_.component_.theta),
                                                                                        _higher_PartialPath->first.pose_, _higher_PartialPath->second.pose_);

    return (escape_timePoint - reference_timePoint).count();
}

bool AA_SIPP::ConflictChecker::conflictSearch(
    const std::string &_higherName, const PartialPath &_higher_PartialPath,
    const std::string &_lowerName, const PartialPath &_lower_PartialPath,
    const Time::TimePoint &_lower_bound, const Time::TimePoint &_upper_bound,
    double _delay)
{
    if (std::fabs((_upper_bound - _lower_bound).count()) < time_precision_)
        return false;

    double key = (_lower_bound + _upper_bound).count() / 2;

    Position::Pose higher_pos = motion_manager_->getPosition(
        _higherName, Time::TimePoint(key), _higher_PartialPath.first.departure_time_,
        _higher_PartialPath.first.pose_, _higher_PartialPath.second.pose_);

    Position::Pose lower_pos = motion_manager_->getPosition(
        _lowerName, Time::TimePoint(key - _delay), _lower_PartialPath.first.departure_time_,
        _lower_PartialPath.first.pose_, _lower_PartialPath.second.pose_);

    double relative_distance = Position::getDistance(higher_pos, lower_pos);

    if (relative_distance > agentSizeDB_[_higherName] + agentSizeDB_[_lowerName] + 1e-8)
    {
        Position::Pose next_higher_pos = motion_manager_->getPosition(
            _higherName, Time::TimePoint(key + time_precision_), _higher_PartialPath.first.departure_time_,
            _higher_PartialPath.first.pose_, _higher_PartialPath.second.pose_);

        Position::Pose next_lower_pos = motion_manager_->getPosition(
            _lowerName, Time::TimePoint(key + time_precision_ - _delay), _lower_PartialPath.first.departure_time_,
            _lower_PartialPath.first.pose_, _lower_PartialPath.second.pose_);

        double next_relative_distance = Position::getDistance(next_higher_pos, next_lower_pos);

        Time::TimePoint new_lower_bound = _lower_bound;
        Time::TimePoint new_upper_bound = _upper_bound;

        if (next_relative_distance - relative_distance > 1e-8)
            new_upper_bound = Time::TimePoint(key);
        else
            new_lower_bound = Time::TimePoint(key);

        return conflictSearch(
            _higherName, _higher_PartialPath,
            _lowerName, _lower_PartialPath,
            new_lower_bound, new_upper_bound,
            _delay);
    }
    else
        return true;
}
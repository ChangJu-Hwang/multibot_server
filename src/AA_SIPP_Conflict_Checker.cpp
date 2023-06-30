#include "multibot_server/AA_SIPP_Conflict_Checker.hpp"

using namespace Low_Level_Engine;

Time::TimeInterval AA_SIPP::ConflictChecker::checkConflict(
    const Path::SinglePath &_higherPath, const Path::SinglePath &_lowerPath)
{
    auto higher_PartialPath = _higherPath.nodes_.begin();
    auto lower_PartialPath = _lowerPath.nodes_.begin();

    Time::TimeInterval unsafe_interval;
    unsafe_interval.is_safe_ = true;

    Time::TimeInterval first_collision_interval;
    Time::TimeInterval second_collision_interval;

    while (higher_PartialPath != _higherPath.nodes_.end() or lower_PartialPath != _lowerPath.nodes_.end())
    {
        Time::TimeInterval result;
        result.is_safe_ = true;

        if (Position::getDistance(higher_PartialPath->first.pose_, higher_PartialPath->second.pose_) +
                Position::getDistance(lower_PartialPath->first.pose_, lower_PartialPath->second.pose_) >
            Position::getDistance(higher_PartialPath->first.pose_, lower_PartialPath->first.pose_) -
                (agentSizeDB_[_higherPath.agentName_] + agentSizeDB_[_lowerPath.agentName_] + 1e-8))
        {
            result = checkPartialConflict(
                _higherPath.agentName_, higher_PartialPath,
                _lowerPath.agentName_, lower_PartialPath);

            if (not(result.is_safe_) or not(unsafe_interval.is_safe_))
            {
                // First Update for unsafe_interval
                if ((unsafe_interval.is_safe_))
                    unsafe_interval = result;

                Position::Pose lower_collision_startPose = motion_manager_->getPosition(
                    _lowerPath.agentName_, unsafe_interval.startTime_, lower_PartialPath->first.departure_time_,
                    lower_PartialPath->first.pose_, lower_PartialPath->second.pose_);

                Time::TimeInterval collsion_interval_start = getCollisionInterval(
                    unsafe_interval.startTime_, agentSizeDB_[_higherPath.agentName_] + agentSizeDB_[_lowerPath.agentName_],
                    _higherPath, higher_PartialPath,
                    lower_collision_startPose);

                bool completeness =
                    std::fabs((_higherPath.nodes_.back().second.arrival_time_ - result.endTime_).count()) > time_precision_ and
                    std::fabs((_lowerPath.nodes_.back().second.arrival_time_ - result.endTime_).count()) > time_precision_;

                if (completeness)
                {
                    unsafe_interval.endTime_ = result.endTime_;
                    // 여기서 도착지점에 대한 interval 계산
                    Position::Pose lower_collision_endPose = motion_manager_->getPosition(
                        _lowerPath.agentName_, unsafe_interval.endTime_, lower_PartialPath->first.departure_time_,
                        lower_PartialPath->first.pose_, lower_PartialPath->second.pose_);
                    
                    Time::TimeInterval collision_interval_end = getCollisionInterval(
                        unsafe_interval.endTime_, agentSizeDB_[_higherPath.agentName_] + agentSizeDB_[_lowerPath.agentName_],
                        _higherPath, higher_PartialPath,
                        lower_collision_endPose);

                    // Interval Shifting
                    collision_interval_end.startTime_ = collision_interval_end.startTime_ - (unsafe_interval.endTime_ - unsafe_interval.startTime_);
                    collision_interval_end.endTime_   = collision_interval_end.endTime_ - (unsafe_interval.endTime_ - unsafe_interval.startTime_);

                    return unsafe_interval;
                }
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

    if (not(unsafe_interval.is_safe_))
        return unsafe_interval;

    return Time::TimeInterval(
        std::max(_higherPath.nodes_.front().first.departure_time_, _lowerPath.nodes_.front().first.departure_time_),
        std::min(_higherPath.nodes_.back().second.arrival_time_, _lowerPath.nodes_.back().second.arrival_time_),
        true);
}

Time::TimeInterval AA_SIPP::ConflictChecker::checkPartialConflict(
    const std::string &_higherName, std::vector<PartialPath>::const_iterator _higher_PartialPath,
    const std::string &_lowerName, std::vector<PartialPath>::const_iterator _lower_PartialPath)
{
    Time::TimePoint lower_bound = std::max(_higher_PartialPath->first.departure_time_, _lower_PartialPath->first.departure_time_);
    Time::TimePoint upper_bound = std::min(_higher_PartialPath->second.arrival_time_, _lower_PartialPath->second.arrival_time_);

    return findCriticalInterval(
        _higherName, *_higher_PartialPath,
        _lowerName, *_lower_PartialPath,
        lower_bound, upper_bound);
}

Time::TimeInterval AA_SIPP::ConflictChecker::findCriticalInterval(
    const std::string &_higherName, const PartialPath &_higher_PartialPath,
    const std::string &_lowerName, const PartialPath &_lower_PartialPath,
    const Time::TimePoint &_lower_bound, const Time::TimePoint &_upper_bound)
{
    if (std::fabs((_upper_bound - _lower_bound).count()) < time_precision_)
        return Time::TimeInterval(
            (_lower_bound + _upper_bound) / 2, (_lower_bound + _upper_bound) / 2,
            true);

    double key = (_lower_bound + _upper_bound).count() / 2;

    Position::Pose higher_pos = motion_manager_->getPosition(
        _higherName, Time::TimePoint(key), _higher_PartialPath.first.departure_time_,
        _higher_PartialPath.first.pose_, _higher_PartialPath.second.pose_);

    Position::Pose lower_pos = motion_manager_->getPosition(
        _lowerName, Time::TimePoint(key), _lower_PartialPath.first.departure_time_,
        _lower_PartialPath.first.pose_, _lower_PartialPath.second.pose_);

    double relative_distance = Position::getDistance(higher_pos, lower_pos);

    if (relative_distance > agentSizeDB_[_higherName] + agentSizeDB_[_lowerName] + 1e-8)
    {
        Position::Pose next_higher_pos = motion_manager_->getPosition(
            _higherName, Time::TimePoint(key + time_precision_), _higher_PartialPath.first.departure_time_,
            _higher_PartialPath.first.pose_, _higher_PartialPath.second.pose_);

        Position::Pose next_lower_pos = motion_manager_->getPosition(
            _lowerName, Time::TimePoint(key + time_precision_), _lower_PartialPath.first.departure_time_,
            _lower_PartialPath.first.pose_, _lower_PartialPath.second.pose_);

        double next_relative_distance = Position::getDistance(next_higher_pos, next_lower_pos);

        Time::TimePoint new_lower_bound = _lower_bound;
        Time::TimePoint new_upper_bound = _upper_bound;

        if (next_relative_distance - relative_distance > 1e-8)
            new_upper_bound = Time::TimePoint(key);
        else
            new_lower_bound = Time::TimePoint(key);

        return findCriticalInterval(
            _higherName, _higher_PartialPath,
            _lowerName, _lower_PartialPath,
            new_lower_bound, new_upper_bound);
    }
    else
    {
        Time::TimePoint critical_lower_bound = binarySearch(
            SEARCH::LEFT,
            _higherName, _higher_PartialPath,
            _lowerName, _lower_PartialPath,
            _lower_bound, Time::TimePoint(key));

        Time::TimePoint critical_upper_bound = binarySearch(
            SEARCH::RIGHT,
            _higherName, _higher_PartialPath,
            _lowerName, _lower_PartialPath,
            Time::TimePoint(key), _upper_bound);

        return Time::TimeInterval(
            critical_lower_bound, critical_upper_bound, false);
    }
}

Time::TimePoint AA_SIPP::ConflictChecker::binarySearch(
    bool _mode,
    const std::string &_higherName, const PartialPath &_higher_PartialPath,
    const std::string &_lowerName, const PartialPath &_lower_PartialPath,
    const Time::TimePoint &_lower_bound, const Time::TimePoint &_upper_bound)
{
    if (std::fabs((_upper_bound - _lower_bound).count()) < time_precision_)
        return Time::TimePoint((_lower_bound + _upper_bound) / 2);

    double key = (_lower_bound + _upper_bound).count() / 2;

    Position::Pose higher_pos = motion_manager_->getPosition(
        _higherName, Time::TimePoint(key), _higher_PartialPath.first.departure_time_,
        _higher_PartialPath.first.pose_, _higher_PartialPath.second.pose_);

    Position::Pose lower_pos = motion_manager_->getPosition(
        _lowerName, Time::TimePoint(key), _lower_PartialPath.first.departure_time_,
        _lower_PartialPath.first.pose_, _lower_PartialPath.second.pose_);

    double relative_distance = Position::getDistance(higher_pos, lower_pos);

    Time::TimePoint new_lower_bound = _lower_bound;
    Time::TimePoint new_upper_bound = _upper_bound;
    if (relative_distance > agentSizeDB_[_higherName] + agentSizeDB_[_lowerName] + 1e-8)
    {
        if (_mode == SEARCH::LEFT)
            new_lower_bound = Time::TimePoint(key);
        else
            new_upper_bound = Time::TimePoint(key);
    }
    else
    {
        if (_mode == SEARCH::LEFT)
            new_upper_bound = Time::TimePoint(key);
        else
            new_lower_bound = Time::TimePoint(key);
    }

    return binarySearch(
        _mode,
        _higherName, _higher_PartialPath,
        _lowerName, _lower_PartialPath,
        new_lower_bound, new_upper_bound);
}

Time::TimeInterval AA_SIPP::ConflictChecker::getCollisionInterval(
    const Time::TimePoint _time, const double _safe_distance,
    const Path::SinglePath &_higherPath, std::vector<PartialPath>::const_iterator _higher_PartialPath,
    const Position::Pose &_lower_pose)
{
    auto partialPath = _higher_PartialPath;

    Position::Pose existing_pose = motion_manager_->getPosition(
        _higherPath.agentName_, _time, partialPath->first.departure_time_,
        partialPath->first.pose_, partialPath->second.pose_);

    Position::Coordinates existing_vector(
        _lower_pose.component_.x - existing_pose.component_.x,
        _lower_pose.component_.y - existing_pose.component_.y);

    Position::Coordinates path_unit_vector(
            partialPath->second.pose_.component_.x - partialPath->first.pose_.component_.x,
            partialPath->second.pose_.component_.y - partialPath->first.pose_.component_.y);
        path_unit_vector = path_unit_vector / path_unit_vector.norm();

    bool search_direction;
    if (existing_vector * path_unit_vector > 0)
        search_direction = DIRECTION::FORWARD;
    else
        search_direction = DIRECTION::REVERSE;

    Position::Coordinates boundary_point;
    Time::TimePoint boundary_time;
    do
    {
        if (partialPath == _higherPath.nodes_.end())
            return Time::TimeInterval(_time, Time::TimePoint::max(), false);
     
        Position::Coordinates reference_point;
        Position::Coordinates direction_unit_vector(
            partialPath->second.pose_.component_.x - partialPath->first.pose_.component_.x,
            partialPath->second.pose_.component_.y - partialPath->first.pose_.component_.y);
        direction_unit_vector = direction_unit_vector / direction_unit_vector.norm();

        if (search_direction == DIRECTION::FORWARD)
        {
            reference_point.x_ = partialPath->first.pose_.component_.x;
            reference_point.y_ = partialPath->first.pose_.component_.y;

            boundary_point.x_ = partialPath->second.pose_.component_.x;
            boundary_point.y_ = partialPath->second.pose_.component_.y;
        }
        else if (search_direction == DIRECTION::REVERSE)
        {
            boundary_point.x_ = partialPath->first.pose_.component_.x;
            boundary_point.y_ = partialPath->first.pose_.component_.y;

            reference_point.x_ = partialPath->second.pose_.component_.x;
            reference_point.y_ = partialPath->second.pose_.component_.y;

            direction_unit_vector = direction_unit_vector * -1;
        }

        Position::Coordinates reference_vector(
            _lower_pose.component_.x - reference_point.x_,
            _lower_pose.component_.y - reference_point.y_);

        double distance = reference_vector * direction_unit_vector
            + std::sqrt(_safe_distance * _safe_distance
                        - std::pow(Position::crossProduct(reference_vector, direction_unit_vector),2));

        Position::Coordinates collision_boundary_point = reference_point + distance * direction_unit_vector;
        Position::Pose collision_boundary_pose(
                collision_boundary_point.x_, collision_boundary_point.y_, partialPath->second.pose_.component_.theta);

        if (Position::getDistance(collision_boundary_pose, partialPath->first.pose_) < 1e-8)
            collision_boundary_pose.component_.theta = partialPath->first.pose_.component_.theta;

        boundary_time = partialPath->first.departure_time_ + motion_manager_->getPartialMoveTime(
            _higherPath.agentName_, collision_boundary_pose,
            partialPath->first.pose_, partialPath->second.pose_);      

        if (search_direction = DIRECTION::FORWARD)
            ++partialPath;
        else if (search_direction = DIRECTION::REVERSE)
            --partialPath;
    } while(Position::getDistance(boundary_point, Position::Coordinates(_lower_pose.component_.x, _lower_pose.component_.y)) < 
            _safe_distance + 1e-8);

    return Time::TimeInterval(std::min(_time, boundary_time) - Time::TimePoint(time_precision_),
                              std::max(_time, boundary_time) + Time::TimePoint(time_precision_), false);
}
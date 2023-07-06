#include "multibot_server/AA_SIPP_ReservationTable.hpp"

using namespace Low_Level_Engine;

void AA_SIPP::ReservationTable::init(double _inflation_radius)
{
    try
    {
        if (std::isnan(map_.property_.width_) or std::isnan(map_.property_.height_))
            throw std::runtime_error("ReservationTable::initReservationTable: Invalid Table Size(NaN)");
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::abort();
    }

    Time::TimeLine initial_timeLine;
    initial_timeLine.interval_list_.clear();
    initial_timeLine.interval_list_.push_back(
        Time::TimeInterval(Time::TimePoint::zero(), Time::TimePoint::max(), true));

    reservationTable_.clear();
    reservationTable_.resize(
        map_.property_.width_,
        std::vector<Time::TimeLine>(map_.property_.height_, initial_timeLine));

    std::vector<std::vector<MapInstance::Cell>> inflated_map = map_.inflate(_inflation_radius);

    for (const auto &row : inflated_map)
    {
        for (const auto &cell : row)
        {
            Position::Index index = cell.idx_;
            reservationTable_[index.x_][index.y_].idx_ = index;
            reservationTable_[index.x_][index.y_].occupied_ = cell.occupied_;
            reservationTable_[index.x_][index.y_].interval_list_.clear();
            Time::TimeInterval interval(
                Time::TimePoint::max(), Time::TimePoint::max(), false);

            if (not(cell.occupied_))
            {
                interval.startTime_ = Time::TimePoint::zero();
                interval.is_safe_ = true;
            }
            reservationTable_[index.x_][index.y_].interval_list_.push_back(interval);
        }
    }
}

bool AA_SIPP::ReservationTable::applySinglePath(
    const std::string &_agentName, const Path::SinglePath &_singlePath)
{
    if (_agentName == _singlePath.agentName_)
        return false;
    
    double inflation_radius = agentSizeDB_[_agentName] + agentSizeDB_[_singlePath.agentName_];

    for (auto nodePairIter = _singlePath.nodes_.begin(); nodePairIter != _singlePath.nodes_.end(); ++nodePairIter)
    {
        std::vector<Position::Index> routeArea = map_utility_->getRouteComponents(
            nodePairIter->first.pose_, nodePairIter->second.pose_);
        
        std::vector<Position::Index> sweepArea = map_.getInflatedArea(
            routeArea, inflation_radius);

        for (const auto &index : sweepArea)
        {
            Position::Index conflict_startIndex, conflict_endIndex;
            std::tie(conflict_startIndex, conflict_endIndex) = conflict_checker_->getConflictScope(
                index, routeArea, inflation_radius);
            
            Time::TimePoint conflict_startTime = nodePairIter->first.departure_time_ + motion_manager_->getPartialMoveTime(
                _singlePath.agentName_, conflict_startIndex,
                nodePairIter->first.pose_, nodePairIter->second.pose_);
            
            Time::TimePoint conflict_endTime = Time::TimePoint::max();
            
            auto nextNodePairIter = nodePairIter;
            auto nextRouteArea = routeArea;
            do
            {
                if (nextNodePairIter != nodePairIter)
                {
                    nextRouteArea = map_utility_->getRouteComponents(
                        nextNodePairIter->first.pose_, nextNodePairIter->second.pose_);
                    conflict_endIndex = conflict_checker_->getConflictScope(
                        index, nextRouteArea, inflation_radius).second;
                }

                if (conflict_endIndex != nextRouteArea.back())
                {
                    conflict_endTime = nextNodePairIter->first.departure_time_;
                    
                    if (conflict_endIndex != Position::Index())
                        conflict_endTime = conflict_endTime + motion_manager_->getPartialMoveTime(
                            _singlePath.agentName_, conflict_endIndex,
                            nextNodePairIter->first.pose_, nextNodePairIter->second.pose_);
                    
                    break;
                }
                else
                    ++nextNodePairIter;
            } while(nextNodePairIter != _singlePath.nodes_.end());

            if (not(addCollisionInterval(index, conflict_startTime, conflict_endTime)))
            {
                std::cerr << "[Error] " << "AA_SIPP_ReservationTable::applySinglePath: "
                          << "Invalid Time Interval" << std::endl;
                std::abort();
            }
        }
    }

    return true;
}

bool AA_SIPP::ReservationTable::isValidMove(
    const std::string &_agentName, const Time::TimePoint &_departure_time,
    const Position::Pose &_from, const Position::Pose &_to)
{
    std::vector<Position::Index> routeComponents = map_utility_->getRouteComponents(
        _from, _to);

    for (const auto &index : routeComponents)
    {
        if (reservationTable_[index.x_][index.y_].occupied_)
            return false;

        Time::TimePoint arrivalTime = _departure_time + motion_manager_->getPartialMoveTime(
                                                            _agentName, index, _from, _to);

        if (arrivalTime + std::numeric_limits<Time::TimePoint>::epsilon() >
            reservationTable_[index.x_][index.y_].interval_list_.back().endTime_)
            return false;
    }

    return true;
}

const std::list<Time::TimeInterval> AA_SIPP::ReservationTable::getSafeIntervals(const Position::Index &_index) const
{
    return reservationTable_[_index.x_][_index.y_].interval_list_;
}   

const std::list<Time::TimeInterval> AA_SIPP::ReservationTable::getSafeIntervals(const Position::Pose &_pose) const
{
    Position::Index index = map_utility_->convertPoseToIndex(_pose);

    return getSafeIntervals(index);
}

bool AA_SIPP::ReservationTable::addCollisionInterval(
    const Position::Index &_index,
    const Time::TimePoint &_startTime, const Time::TimePoint &_endTime)
{
    if (_startTime + std::numeric_limits<Time::TimePoint>::epsilon() > _endTime)
        return false;

    Time::TimeLine &timeLine = reservationTable_[_index.x_][_index.y_];

    if (timeLine.occupied_)
        return true;

    for (auto interval = timeLine.interval_list_.begin(); interval != timeLine.interval_list_.end();)
    {
        if (not(interval->is_safe_))
            interval = timeLine.interval_list_.erase(interval);

        // Collision Start >= Safe End
        if (_startTime + std::numeric_limits<Time::TimePoint>::epsilon() >= interval->endTime_)
            ++interval;

        // Safe Start >= Collision End
        else if (interval->startTime_ + std::numeric_limits<Time::TimePoint>::epsilon() >= _endTime)
            break;

        // (Collision start > Safe start) and (Collision end >= Safe end)
        else if (
            _startTime > interval->startTime_ + std::numeric_limits<Time::TimePoint>::epsilon() and
            _endTime + std::numeric_limits<Time::TimePoint>::epsilon() >= interval->endTime_)
        {
            interval->endTime_ = _startTime;
            ++interval;
        }

        // (Safe start >= Collision start) and (Safe end > Collision end)
        else if (
            interval->startTime_ + std::numeric_limits<Time::TimePoint>::epsilon() >= _startTime and
            interval->endTime_ > _endTime + std::numeric_limits<Time::TimePoint>::epsilon())
        {
            interval->startTime_ = _endTime;
            break;
        }

        // (Collision start > Safe Start) and (Safe end > Collision end)
        else if (
            _startTime > interval->startTime_ + std::numeric_limits<Time::TimePoint>::epsilon() and
            interval->endTime_ > _endTime + std::numeric_limits<Time::TimePoint>::epsilon())
        {
            Time::TimeInterval newInterval;

            newInterval.startTime_ = interval->startTime_;
            newInterval.endTime_ = _startTime;
            newInterval.is_safe_ = true;

            timeLine.interval_list_.insert(interval, newInterval);
            interval->startTime_ = _endTime;

            break;
        }

        // (Safe start >= Collision start) and (Collision end >= Safe end)
        else
            interval = timeLine.interval_list_.erase(interval);
    }

    return true;
}
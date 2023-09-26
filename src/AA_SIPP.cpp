#include "multibot_server/AA_SIPP.hpp"

#include <future>

using namespace Low_Level_Engine;

std::pair<Traj::SingleTraj, bool> AA_SIPP::Planner::search(
    const std::string &_agentName,
    const std::vector<std::string> &_higher_agents,
    const Traj::TrajSet &_trajSet,
    const std::pair<Position::Coordinates, Position::Coordinates> &_searchSpace,
    const double _timeLimit)
{
    map_utility_->restrictArea(_searchSpace);
    reservation_table_->init(agents_[_agentName].size_);
    higher_trajs_.clear();

    for (const auto &singleTraj : _trajSet)
    {
        if (std::find(_higher_agents.begin(), _higher_agents.end(), singleTraj.second.agentName_) == _higher_agents.end())
            continue;

        reservation_table_->applySingleTraj(_agentName, singleTraj.second);
        higher_trajs_.push_back(singleTraj.second);
    }

    std::list<Time::TimeInterval> starts = {
        reservation_table_->getSafeIntervals(agents_[_agentName].start_).front()};
    std::list<Time::TimeInterval> goals = reservation_table_->getSafeIntervals(
        agents_[_agentName].goal_);

    auto partialTraj = find_partial_traj(
        _agentName, starts, goals, _timeLimit);

    return partialTraj;
}

std::pair<Traj::SingleTraj, bool> AA_SIPP::Planner::find_partial_traj(
    const std::string &_agentName,
    const std::list<Time::TimeInterval> &_starts,
    const std::list<Time::TimeInterval> &_goals,
    const double _timeLimit)
{
    generateRoot(agents_[_agentName].start_, _starts);

    auto search_startTime = std::chrono::system_clock::now();
    Time::TimePoint duration = Time::TimePoint::zero();

    while (not(open_.empty()))
    {
        AA_SIPP::Node curNode = *open_.begin();
        open_.erase(open_.begin());

        if (curNode.idx_ == map_utility_->convertPoseToIndex(agents_[_agentName].goal_) and
            curNode.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon() >= _goals.back().startTime_)
            return std::make_pair(constructSingleTraj(_agentName, curNode), true);

        if (duration.count() > _timeLimit)
            return std::make_pair(Traj::SingleTraj(), false);

        if (close_.find(curNode) != close_.end())
            continue;

        auto parent = &close_.insert(close_.end(), std::make_pair(curNode, curNode))->second;
        std::vector<Position::Index> neighborIndexes = map_utility_->getNeighborIndex(parent->pose_);
        neighborIndexes = map_utility_->getValidIndexes(_agentName, neighborIndexes);

        std::vector<AA_SIPP::Node> successors;
        successors.clear();

        for (const auto &neighborIndex : neighborIndexes)
        {
            auto result = getSuccessors(
                _agentName, neighborIndex, *parent);

            std::for_each(result.begin(), result.end(),
                          [&](AA_SIPP::Node &_successor)
                          { _successor.parent_ = parent; });

            successors.insert(
                successors.end(), result.begin(), result.end());

            if (parent->parent_ != nullptr)
            {
                auto parent_result = getSuccessors(
                    _agentName, neighborIndex, *parent->parent_);

                std::for_each(parent_result.begin(), parent_result.end(),
                              [&](AA_SIPP::Node &_parent_successor)
                              { _parent_successor.parent_ = parent->parent_; });

                successors.insert(
                    successors.end(), parent_result.begin(), parent_result.end());
            }
        }

        for (const auto &successor : successors)
        {
            if (close_.find(successor) != close_.end())
                continue;

            auto iter_open = std::find_if(open_.begin(), open_.end(),
                                          [&](const AA_SIPP::Node &_node)
                                          { return _node == successor; });

            if (iter_open != open_.end())
            {
                if (successor.gVal_ + successor.hVal_ + 1e-8 >= iter_open->gVal_ + iter_open->hVal_)
                    continue;

                open_.erase(iter_open);
            }

            open_.insert(successor);
        }
        duration = std::chrono::system_clock::now() - search_startTime;
    }

    std::cout << "No result" << std::endl;
    return std::make_pair(Traj::SingleTraj(), false);
}

void AA_SIPP::Planner::generateRoot(
    const Position::Pose &_startPose,
    const std::list<Time::TimeInterval> &_starts)
{
    open_.clear();
    close_.clear();

    for (const auto &safe_interval : _starts)
    {
        AA_SIPP::Node startNode;

        startNode.pose_ = _startPose;
        startNode.idx_ = map_utility_->convertPoseToIndex(_startPose);

        startNode.gVal_ = 0;
        startNode.hVal_ = 0;

        startNode.safe_interval_ = safe_interval;
        startNode.parent_ = nullptr;

        open_.insert(startNode);
    }
}

std::vector<AA_SIPP::Node> AA_SIPP::Planner::getSuccessors(
    const std::string &_agentName,
    const Position::Index &_index, const AA_SIPP::Node &_parentNode)
{
    double theta = std::atan2(
        _index.y_ - _parentNode.idx_.y_,
        _index.x_ - _parentNode.idx_.x_);
    while (std::fabs(theta) - M_PI > 1e-8 or
           theta + 1e-8 > M_PI)
    {
        int sign = theta > 0 ? 1 : -1;
        theta = theta - sign * 2 * M_PI;
    }

    AA_SIPP::Node childNode;

    childNode.idx_ = _index;
    childNode.pose_ = map_utility_->convertIndexToPose(
        _index, theta);

    Time::TimePoint moveTime = motion_manager_->getMoveTime(
        _agentName, _parentNode.pose_, childNode.pose_);

    childNode.arrival_time_ = _parentNode.arrival_time_ + moveTime;

    if (not(reservation_table_->isValidMove(
            _agentName, _parentNode.arrival_time_,
            _parentNode.pose_, childNode.pose_)))
    {
        return std::vector<AA_SIPP::Node>();
    }

    std::vector<AA_SIPP::Node> successors;
    successors.clear();

    std::list<Time::TimeInterval> safe_intervals = reservation_table_->getSafeIntervals(_index);
    for (const auto &safe_interval : safe_intervals)
    {
        childNode.safe_interval_ = safe_interval;

        if (close_.find(childNode) != close_.end())
            continue;

        if (childNode.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon() >= safe_interval.endTime_)
            continue;

        if (safe_interval.startTime_ > childNode.arrival_time_ + std::numeric_limits<Time::TimePoint>::epsilon())
            childNode.arrival_time_ = safe_interval.startTime_;

        double delay = computeTotalDelays(
            _agentName, _parentNode.pose_, childNode.pose_,
            _parentNode.arrival_time_, safe_interval.endTime_ - _parentNode.arrival_time_);

        if (childNode.arrival_time_ - moveTime + Time::TimePoint(delay) + std::numeric_limits<Time::TimePoint>::epsilon() >= _parentNode.safe_interval_.endTime_)
            continue;

        if (childNode.arrival_time_ + Time::TimePoint(delay) + std::numeric_limits<Time::TimePoint>::epsilon() >= childNode.safe_interval_.endTime_)
            continue;

        childNode.arrival_time_ = childNode.arrival_time_ + Time::TimePoint(delay);
        childNode.gVal_ = childNode.arrival_time_.count();
        childNode.updateHeuristic(agents_[_agentName].goal_, agents_[_agentName].max_linVel_);

        successors.push_back(childNode);
    }

    return successors;
}

Traj::SingleTraj AA_SIPP::Planner::constructSingleTraj(
    const std::string &_agentName, const AA_SIPP::Node &_goalNode)
{
    Traj::SingleTraj singleTraj;

    singleTraj.agentName_ = _agentName;
    singleTraj.nodes_.clear();

    AA_SIPP::Node curNode = _goalNode;
    Traj::SingleTraj::Node localGoalNode;

    localGoalNode.pose_ = curNode.pose_;
    localGoalNode.arrival_time_ = curNode.arrival_time_;
    localGoalNode.departure_time_ = curNode.arrival_time_;

    while (curNode.parent_ != nullptr)
    {
        Traj::SingleTraj::Node localStartNode;

        localStartNode.pose_ = curNode.parent_->pose_;
        localStartNode.arrival_time_ = curNode.parent_->arrival_time_;
        localStartNode.departure_time_ = localGoalNode.arrival_time_ - motion_manager_->getMoveTime(
                                                                           _agentName, localStartNode.pose_, localGoalNode.pose_);

        singleTraj.nodes_.push_back(std::make_pair(localStartNode, localGoalNode));

        curNode = *curNode.parent_;
        localGoalNode = localStartNode;
    }
    std::reverse(singleTraj.nodes_.begin(), singleTraj.nodes_.end());

    Traj::SingleTraj::Node rotationNode;

    rotationNode.pose_ = agents_[_agentName].goal_;
    rotationNode.arrival_time_ = singleTraj.nodes_.back().second.departure_time_ + motion_manager_->getMoveTime(
                                                                                      _agentName, singleTraj.nodes_.back().second.pose_, rotationNode.pose_);
    rotationNode.departure_time_ = Time::TimePoint::max();

    singleTraj.nodes_.push_back(std::make_pair(singleTraj.nodes_.back().second, rotationNode));
    singleTraj.cost_ = rotationNode.arrival_time_.count();

    return singleTraj;
}

double AA_SIPP::Planner::computeTotalDelays(
    const std::string &_agentName,
    const Position::Pose &_startPose, const Position::Pose &_goalPose,
    const Time::TimePoint &_departure_time, const Time::TimePoint &_max_delay_limit)
{
    Traj::SingleTraj::Node start, goal;

    start.pose_ = _startPose;
    start.departure_time_ = _departure_time;

    goal.pose_ = _goalPose;
    goal.arrival_time_ = _departure_time + motion_manager_->getMoveTime(
                                               _agentName, _startPose, _goalPose);

    Traj::SingleTraj lower_traj;
    lower_traj.nodes_.clear();

    lower_traj.agentName_ = _agentName;
    lower_traj.nodes_.push_back({start, goal});

    double total_delay = 0.0, delay = 0.0;
    do
    {
        lower_traj.nodes_.front().first.departure_time_ += Time::TimePoint(delay);
        lower_traj.nodes_.front().second.arrival_time_ += Time::TimePoint(delay);

        std::vector<std::future<double>> delayTimeThreads;
        delayTimeThreads.clear();

        for (const auto &higher_traj : higher_trajs_)
        {
            delayTimeThreads.push_back(
                std::async(std::launch::async, [this, higher_traj, lower_traj]() -> double
                           { return this->conflict_checker_->getDelayTime(higher_traj, lower_traj); }));
        }

        delay = 0.0;
        for (auto &delayTimeThread : delayTimeThreads)
        {
            delay = std::max(delay, delayTimeThread.get());
        }
        total_delay += delay;

        if (total_delay > _max_delay_limit.count() + 1e-8)
            break;

        if (std::fabs(total_delay - std::numeric_limits<double>::max()) < 1e-3)
            break;

    } while (delay > 1e-3);

    return total_delay;
}

AA_SIPP::Planner::Planner(std::shared_ptr<Instance_Manager> &_instance_manager)
{
    _instance_manager->attach(*this);

    _instance_manager->attach(*(map_utility_));
    _instance_manager->attach(*(motion_manager_));
    _instance_manager->attach(*(conflict_checker_));
    _instance_manager->attach(*(reservation_table_));
}
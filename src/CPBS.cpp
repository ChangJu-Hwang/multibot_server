#include "multibot_server/CPBS.hpp"

#include <future>

using namespace High_Level_Engine;

std::pair<Traj::TrajSet, bool> CPBS::Solver::solve()
{
    if (not(generateRoot()))
    {
        std::cerr << "[Error] "
                  << "Generate Root Failed" << std::endl;
        return std::make_pair(Traj::TrajSet(), false);
    }

    while (not(open_.empty()))
    {
        Node *curNode = open_.top();
        open_.pop();

        updateNode(curNode);

        if (curNode->conflicts_.empty())
        {
            if (validateResult() == false)
            {
                std::cerr << "\n"
                          << "[Error] CPBS: Invalid Solution" << std::endl;

                return std::make_pair(Traj::TrajSet(), false);
            }

            std::cout << "\n"
                      << "[CPBS] Solution Found" << std::endl;

            return std::make_pair(trajSet_, true);
        }

        curNode->conflict_ = curNode->conflicts_.back();
        
        auto firstChildResult = generateChild(
            curNode, curNode->conflict_.first, curNode->conflict_.second);

        auto secondChildResult = generateChild(
            curNode, curNode->conflict_.second, curNode->conflict_.first);

        if (firstChildResult.second == true and secondChildResult.second == true)
        {
            if (firstChildResult.first->cost_ > secondChildResult.first->cost_)
            {
                open_.push(firstChildResult.first);
                open_.push(secondChildResult.first);
            }
            else
            {
                open_.push(secondChildResult.first);
                open_.push(firstChildResult.first);
            }
        }
        else if (firstChildResult.second == false and secondChildResult.second == false)
        {
            if (curNode->ignoredConflictNum_ < curNode->conflicts_.size())
            {
                curNode->ignoredConflictNum_++;
                curNode->conflicts_.push_front(curNode->conflict_);
                curNode->conflicts_.erase(std::prev(curNode->conflicts_.end(), 1));
                open_.push(curNode);
            }
            else
            {
                std::cout << "\n"
                          << "[CPBS] No Solution" << std::endl;

                return std::make_pair(Traj::TrajSet(), false);
            }
        }
        else if (firstChildResult.second == false)
            open_.push(secondChildResult.first);
        else if (secondChildResult.second == false)
            open_.push(firstChildResult.first);
    }

    std::cout << "\n"
              << "[CPBS] No Solution" << std::endl;

    return std::make_pair(Traj::TrajSet(), false);
}

std::pair<Position::Coordinates, Position::Coordinates> CPBS::Solver::restrict_searchSpace()
{
    Position::Coordinates lower_left(
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

    Position::Coordinates upper_right(
        -1 * std::numeric_limits<double>::infinity(), -1 * std::numeric_limits<double>::infinity());

    for (const auto &agent : agents_)
    {
        lower_left.x_ = std::min(agent.second.start_.component_.x, lower_left.x_);
        lower_left.y_ = std::min(agent.second.start_.component_.y, lower_left.y_);

        lower_left.x_ = std::min(agent.second.goal_.component_.x, lower_left.x_);
        lower_left.y_ = std::min(agent.second.goal_.component_.y, lower_left.y_);

        upper_right.x_ = std::max(agent.second.start_.component_.x, upper_right.x_);
        upper_right.y_ = std::max(agent.second.start_.component_.y, upper_right.y_);

        upper_right.x_ = std::max(agent.second.goal_.component_.x, upper_right.x_);
        upper_right.y_ = std::max(agent.second.goal_.component_.y, upper_right.y_);
    }

    return {lower_left, upper_right};
}

bool CPBS::Solver::generateRoot()
{
    auto root = new Node;

    root->parent_ = nullptr;
    root->cost_ = 0.0;
    root->makeSpan_ = Time::TimePoint::zero();

    root->trajSet_.clear();

    std::vector<std::future<std::tuple<std::string, std::string, double>>> delayTimeThreads;
    delayTimeThreads.clear();

    for (const auto &agent : agents_)
    {
        std::cout << "[" << agent.second.name_ << "] "
                  << agent.second.start_ << " -> " << agent.second.goal_ << std::endl;
        auto trajResult = planner_->search(
            agent.second.name_, std::vector<std::string>(), Traj::TrajSet());

        if (trajResult.second == false)
            return false;

        root->cost_ = root->cost_ + trajResult.first.cost_;
        root->makeSpan_ = std::max(
            trajResult.first.nodes_.back().second.arrival_time_, root->makeSpan_);

        for (const auto &traj : root->trajSet_)
        {
            delayTimeThreads.push_back(
                std::async(std::launch::async, [this, traj, trajResult]() -> std::tuple<std::string, std::string, double>
                           { double delayTime = this->conflict_checker_->getDelayTime(traj.second, trajResult.first);
                             return std::make_tuple(traj.second.agentName_, trajResult.first.agentName_, delayTime); }));
        }

        root->trajSet_.insert(std::make_pair(agent.second.name_, trajResult.first));
    }

    root->conflicts_.clear();
    for (auto &delayTimeThread : delayTimeThreads)
    {
        auto conflictResult = delayTimeThread.get();

        if (std::get<2>(conflictResult) < 1e-3)
            continue;

        root->conflicts_.push_back(std::make_pair(
            std::get<0>(conflictResult), std::get<1>(conflictResult)));
    }
    delayTimeThreads.clear();

    open_.push(root);

    return true;
}

std::pair<CPBS::Node *, bool> CPBS::Solver::generateChild(
    Node *_parent, const std::string &_low, const std::string &_high)
{
    auto childNode = initChild(_parent, _low, _high);
    Traj::TrajSet trajSet = trajSet_;

    std::vector<std::string> ordered_agents;
    topologicalSort(ordered_agents);

    std::priority_queue<std::vector<std::string>::iterator> to_replan;
    to_replan.emplace(std::find(ordered_agents.begin(), ordered_agents.end(), _low));

    std::vector<std::string> higher_agents = getHigherPriorityAgents(_high, ordered_agents);
    higher_agents.push_back(_high);
    std::vector<std::string> lower_agents = getLowerPriorityAgents(_low, ordered_agents);

    // update lookup_table
    std::unordered_map<std::string, bool> lookup_table;
    lookup_table.clear();
    for (const auto &conflict : childNode->conflicts_)
    {
        std::string lower = conflict.first, higher = conflict.second;

        if (lower == _low or higher == _low)
            continue;

        if (priority_graph_.find(std::make_pair(lower, higher)) != priority_graph_.end())
            std::swap(lower, higher);

        if (std::find(higher_agents.begin(), higher_agents.end(), higher) == higher_agents.end())
            continue;

        if (std::find(lower_agents.begin(), lower_agents.end(), lower) == lower_agents.end())
            continue;

        auto iter = lookup_table.find(lower);
        if (iter != lookup_table.end() and iter->second == true)
            continue;

        to_replan.emplace(std::find(ordered_agents.begin(), ordered_agents.end(), lower));
        lookup_table[lower] = true;
    }

    childNode->trajSet_.clear();
    while (not(to_replan.empty()))
    {
        std::string agentName_to_replan = *to_replan.top();
        to_replan.pop();
        lookup_table[agentName_to_replan] = false;

        std::vector<std::string> highers_than_replan = getHigherPriorityAgents(
            agentName_to_replan, ordered_agents);

        auto result = planner_->search(
            agentName_to_replan, highers_than_replan, trajSet);

        if (result.second == false)
            return std::make_pair(nullptr, false);

        childNode->cost_ = childNode->cost_ - trajSet[agentName_to_replan].cost_ + result.first.cost_;
        trajSet[agentName_to_replan] = result.first;
        childNode->trajSet_.emplace(std::make_pair(agentName_to_replan, result.first));

        for (auto conflict = childNode->conflicts_.begin(); conflict != childNode->conflicts_.end();)
        {
            if (conflict->first == agentName_to_replan)
                conflict = childNode->conflicts_.erase(conflict);
            else if (conflict->second == agentName_to_replan)
                conflict = childNode->conflicts_.erase(conflict);
            else
                ++conflict;
        }

        // find new agent to replan
        std::vector<std::string> lowers_than_replan = getLowerPriorityAgents(
            agentName_to_replan, ordered_agents);
        std::vector<std::future<std::pair<std::string, double>>> delayTimeThreads;
        delayTimeThreads.clear();

        for (const auto &lower_than_replan : lowers_than_replan)
        {
            auto iter = lookup_table.find(lower_than_replan);
            if (iter != lookup_table.end() and iter->second == true)
                continue;

            auto lower_traj = trajSet[lower_than_replan];

            delayTimeThreads.push_back(
                std::async(std::launch::async, [this, result, lower_traj]() -> std::pair<std::string, double>
                           { double delayTime = this->conflict_checker_->getDelayTime(result.first, lower_traj);
                             return std::make_pair(lower_traj.agentName_, delayTime); }));
        }

        for (auto &delayTimeThread : delayTimeThreads)
        {
            auto conflictResult = delayTimeThread.get();

            if (conflictResult.second < 1e-3)
                continue;

            childNode->conflicts_.emplace_back(std::make_pair(
                result.first.agentName_, conflictResult.first));

            to_replan.emplace(std::find(ordered_agents.begin(), ordered_agents.end(), conflictResult.first));

            auto iter = lookup_table.find(conflictResult.first);
            if (iter != lookup_table.end())
                lookup_table[conflictResult.first] = true;
            else
                lookup_table.emplace(std::make_pair(conflictResult.first, true));
        }
        delayTimeThreads.clear();
    }

    return std::make_pair(childNode, true);
}

void CPBS::Solver::updateNode(Node *_curNode)
{
    trajSet_.clear();
    priority_graph_.clear();

    for (auto node = _curNode; node != nullptr; node = node->parent_)
    {
        for (const auto &singleTraj : node->trajSet_)
        {
            if (trajSet_.find(singleTraj.second.agentName_) == trajSet_.end())
                trajSet_.insert(std::make_pair(
                    singleTraj.second.agentName_, singleTraj.second));
        }

        if (node->parent_ != nullptr)
        {
            if (node->higher_agentName_ == node->parent_->conflict_.first)
                priority_graph_.insert(node->parent_->conflict_);

            else if (node->higher_agentName_ == node->parent_->conflict_.second)
                priority_graph_.insert(std::make_pair(
                    node->parent_->conflict_.second, node->parent_->conflict_.first));
        }
    }
}

bool CPBS::Solver::validateResult()
{
    bool validationFlag = true;

    std::vector<std::future<std::tuple<std::string, std::string, double>>> delayTimeThreads;
    delayTimeThreads.clear();

    for (auto firstIter = trajSet_.begin(); firstIter != trajSet_.end(); ++firstIter)
    {
        for (auto secondIter = std::next(firstIter, 1); secondIter != trajSet_.end(); ++secondIter)
        {
            const Traj::SingleTraj firstTraj = firstIter->second, secondTraj = secondIter->second;
            delayTimeThreads.push_back(
                std::async(std::launch::async, [this, firstTraj, secondTraj]() -> std::tuple<std::string, std::string, double>
                           { double delayTime = this->conflict_checker_->getDelayTime(firstTraj, secondTraj);
                             return std::make_tuple(firstTraj.agentName_, secondTraj.agentName_, delayTime); }));
        }
    }

    for (auto &delayTimeThread : delayTimeThreads)
    {
        auto conflictResult = delayTimeThread.get();

        if (std::get<2>(conflictResult) < 1e-3)
            continue;

        validationFlag = false;

        std::cout << "\n"
                  << "[CPBS] Unsolved Conflict: "
                  << std::get<0>(conflictResult) << " & " << std::get<1>(conflictResult)
                  << std::endl;
    }
    delayTimeThreads.clear();

    return validationFlag;
}

CPBS::Node *CPBS::Solver::initChild(
    Node *_parent, const std::string &_low, const std::string &_high)
{
    auto childNode = new Node();

    childNode->parent_ = _parent;
    childNode->cost_ = _parent->cost_;
    childNode->conflicts_ = _parent->conflicts_;
    childNode->higher_agentName_ = _high;

    if (priority_graph_.find(std::make_pair(_low, _high)) != priority_graph_.end())
        priority_graph_.erase(std::make_pair(_low, _high));

    priority_graph_.insert(std::make_pair(_high, _low));

    return childNode;
}

void CPBS::Solver::topologicalSort(std::vector<std::string> &_list)
{
    _list.clear();

    for (const auto &agent : agents_)
    {
        if (std::find(_list.begin(), _list.end(), agent.second.name_) == _list.end())
            topologicalSortUtil(agent.second.name_, _list);
    }

    std::reverse(_list.begin(), _list.end());
}

void CPBS::Solver::topologicalSortUtil(
    const std::string &_agentName, std::vector<std::string> &_list)
{
    for (const auto &other : agents_)
    {
        if (std::find(_list.begin(), _list.end(), other.second.name_) != _list.end())
            continue;

        if (priority_graph_.find(std::make_pair(other.second.name_, _agentName)) != priority_graph_.end())
            topologicalSortUtil(other.second.name_, _list);
    }

    _list.push_back(_agentName);
}

std::vector<std::string> CPBS::Solver::getHigherPriorityAgents(
    const std::string &_target, const std::vector<std::string> &_list)
{
    std::vector<std::string> higherAgents;
    higherAgents.clear();

    for (auto iter = _list.rbegin(); iter != _list.rend(); ++iter)
    {
        if (*iter == _target)
            break;

        higherAgents.push_back(*iter);
    }

    return higherAgents;
}

std::vector<std::string> CPBS::Solver::getLowerPriorityAgents(
    const std::string &_target, const std::vector<std::string> &_list)
{
    std::vector<std::string> lowerAgents;
    lowerAgents.clear();

    for (auto iter = _list.begin(); iter != _list.end(); ++iter)
    {
        if (*iter == _target)
            break;

        lowerAgents.push_back(*iter);
    }

    return lowerAgents;
}

CPBS::Solver::Solver(std::shared_ptr<Instance::Instance_Manager> _instance_manager)
{
    _instance_manager->attach(*(this));

    _instance_manager->attach(*(map_utility_));
    _instance_manager->attach(*(motion_manager_));
    _instance_manager->attach(*(conflict_checker_));

    planner_ = std::make_shared<AA_SIPP::Planner>(_instance_manager);

    std::cout << "CPBS::Solver Constructor" << std::endl;
}
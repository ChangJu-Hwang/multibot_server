#include "multibot_server/Instance_Manager.hpp"

#include <mutex>
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h> // Need to install libyaml-cpp-dev

using namespace Instance;

void Instance_Manager::insertAgent(const std::pair<std::string, AgentInstance::Agent> &_agent)
{
    agents_.insert(_agent);
    notify();
}

void Instance_Manager::deleteAgent(const std::string _agentName)
{
    if (agents_.contains(_agentName))
    {
        agents_.erase(_agentName);
        notify();
    }
}

void Instance_Manager::setGoal(
    const std::string _agentName, const geometry_msgs::msg::Pose2D _goal)
{
    if (agents_.contains(_agentName))
    {
        agents_[_agentName].goal_.component_ = _goal;
        notify();
    }
}

void Instance_Manager::saveMap(const MapInstance::BinaryOccupancyMap &_map)
{
    map_ = _map;
}

void Instance_Manager::exportResult(
    const Path::PathSet &_paths,
    const std::string &_output_fName,
    const std::string &_directory) const
{
    std::filesystem::path resultDirectory("src/multibot_server/" + _directory);
    bool directoryExist = std::filesystem::exists(resultDirectory);
    if (directoryExist)
        std::filesystem::remove_all("src/multibot_server/" + _directory);

    std::filesystem::create_directories("src/multibot_server/" + _directory);

    YAML::Node node;
    for (const auto &singleAgent : _paths)
    {
        YAML::Node agent;
        agent["name"] = singleAgent.second.agentName_;
        agent["type"] = agents_.find(singleAgent.second.agentName_)->second.type_;
        agent["cost"] = singleAgent.second.cost_;

        for (const auto &nodePair : singleAgent.second.nodes_)
        {
            YAML::Node path;
            std::vector<double> start = {nodePair.first.pose_.component_.x,
                                         nodePair.first.pose_.component_.y,
                                         nodePair.first.pose_.component_.theta};
            path["Start"] = start;
            path["Start"].SetStyle(YAML::EmitterStyle::Flow);

            std::vector<double> goal = {nodePair.second.pose_.component_.x,
                                        nodePair.second.pose_.component_.y,
                                        nodePair.second.pose_.component_.theta};
            path["Goal"] = goal;
            path["Goal"].SetStyle(YAML::EmitterStyle::Flow);

            std::vector<double> time_interval = {nodePair.first.departure_time_.count(),
                                                 nodePair.second.arrival_time_.count()};
            path["Time Interval"] = time_interval;
            path["Time Interval"].SetStyle(YAML::EmitterStyle::Flow);

            agent["path"].push_back(path);
        }

        node["Log"].push_back(agent);
    }

    std::string fPath = "src/multibot_server/" + _directory + "/" + _output_fName + ".yaml";
    std::ofstream fOut(fPath);
    fOut << node;
}

void Instance_Manager::attach(Observer::ObserverInterface<InstanceMsg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    if (std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
        list_observer_.push_back(&_observer);
}

void Instance_Manager::detach(Observer::ObserverInterface<InstanceMsg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    auto observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    while (observer_iter != list_observer_.end())
    {
        list_observer_.remove(&_observer);
        observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    }
}

void Instance_Manager::notify()
{
    std::scoped_lock<std::mutex> lock(mtx_);

    Instance::InstanceMsg msg;
    msg.first = agents_;
    msg.second = map_;

    for (auto &observer : list_observer_)
        observer->update(msg);
}
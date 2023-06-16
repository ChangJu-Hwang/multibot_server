#include "server/Instance_Manager.hpp"

#include <mutex>
#include <yaml-cpp/yaml.h>

using namespace Instance;

void Instance_Manager::saveAgents(const std::unordered_map<std::string, AgentInstance::Agent> &_agents)
{
    agents_ = _agents;
    
    for(const auto &agent : agents_)
        std::cout << agent.second << std::endl;
}

void Instance_Manager::saveMap(const MapInstance::BinaryOccupancyMap &_map)
{
    map_    = _map;

    std::cout << map_.property_ << std::endl;
}

void Instance_Manager::attach(Observer::ObserverInterface<InstanceMsg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    if(std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
        list_observer_.push_back(&_observer);
}

void Instance_Manager::detach(Observer::ObserverInterface<InstanceMsg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    auto observer_iter  = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    while(observer_iter != list_observer_.end())
    {
        list_observer_.remove(&_observer);
        observer_iter   = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    }
}

void Instance_Manager::notify()
{
    std::scoped_lock<std::mutex> lock(mtx_);

    Instance::InstanceMsg msg;
    msg.first   = agents_;
    msg.second  = map_;

    for (auto &observer : list_observer_)
        observer->update(msg);
}
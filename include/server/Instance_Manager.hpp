#pragma once

#include "server/Instance.hpp"
#include "server/Interface/Observer_Interface.hpp"

using namespace MAPF_Util;

namespace Instance
{
    typedef std::pair<std::unordered_map<std::string, AgentInstance::Agent>,
                      MapInstance::BinaryOccupancyMap>
        InstanceMsg;

    class Instance_Manager : public Observer::SubjectInterface<InstanceMsg>
    {
    public:
        void saveAgents(const std::unordered_map<std::string, AgentInstance::Agent> &_agents);
        void saveMap(const MapInstance::BinaryOccupancyMap &_map);

    public:
        void attach(Observer::ObserverInterface<InstanceMsg> &_observer) override;
        void detach(Observer::ObserverInterface<InstanceMsg> &_observer) override;
        void notify() override;

    private:
        std::unordered_map<std::string, AgentInstance::Agent> agents_;
        MapInstance::BinaryOccupancyMap map_;

        std::list<Observer::ObserverInterface<InstanceMsg> *> list_observer_;

    public:
        Instance_Manager() {}
        ~Instance_Manager() {}
    }; // class Instance_Manager

} // namespace Instance
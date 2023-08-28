#pragma once

#include "multibot_util/Instance.hpp"
#include "multibot_util/Interface/Observer_Interface.hpp"

using namespace MAPF_Util;

namespace Instance
{
    typedef std::pair<std::unordered_map<std::string, AgentInstance::Agent>,
                      MapInstance::BinaryOccupancyMap>
        InstanceMsg;

    class Instance_Manager : public Observer::SubjectInterface<InstanceMsg>
    {
    public:
        void insertAgent(const std::pair<std::string, AgentInstance::Agent> &_agent);
        void deleteAgent(const std::string _agentName);
        void setStart(const std::string _agentName, const geometry_msgs::msg::Pose2D _start);
        void setGoal(const std::string _agentName, const geometry_msgs::msg::Pose2D _goal);
        void saveMap(const MapInstance::BinaryOccupancyMap &_map);
        void exportResult(
            const Path::PathSet &_paths,
            const std::string &_output_fName = "CPBS_Log",
            const std::string &_directory = "Results") const;

    public:
        void attach(Observer::ObserverInterface<InstanceMsg> &_observer) override;
        void detach(Observer::ObserverInterface<InstanceMsg> &_observer) override;
        void notify() override;

    private:
        std::unordered_map<std::string, AgentInstance::Agent> agents_;
        MapInstance::BinaryOccupancyMap map_;

        std::list<Observer::ObserverInterface<InstanceMsg> *> list_observer_;

    public:
        Instance_Manager() { agents_.clear(); }
        ~Instance_Manager() {}
    }; // class Instance_Manager

} // namespace Instance
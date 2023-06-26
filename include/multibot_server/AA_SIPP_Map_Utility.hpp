#pragma once

#include "multibot_util/Interface/Observer_Interface.hpp"

#include "multibot_server/Instance_Manager.hpp"

using namespace Instance;

namespace Low_Level_Engine
{
    namespace AA_SIPP
    {
        class Map_Utility : public Observer::ObserverInterface<InstanceMsg>
        {
        public:
            std::vector<Position::Index> getNeighborIndex(const Position::Index &_index) const;
            std::vector<Position::Index> getValidIndexes(
                const std::string &_agentName, const std::vector<Position::Index> &_target);
            bool isValidIndexes(
                const std::string &_agentName, const std::vector<Position::Index> &_target);
            const Position::Pose convertIndexToPose(
                const Position::Index &_target,
                const Position::Pose &_from, const Position::Pose &_to) const;

        public:
            void update(const InstanceMsg &_msg)
            {
                for (const auto &agentData : _msg.first)
                {
                    std::pair<std::string, double> agent_size =
                        std::make_pair(agentData.second.name_, agentData.second.size_);
                    
                    agentSizeDB_.insert(agent_size);
                }

                map_ = _msg.second;
            }
        
        private:
            std::unordered_map<std::string, double> agentSizeDB_;
            MapInstance::BinaryOccupancyMap map_;

            std::vector<std::vector<MapInstance::Cell>> inflated_mapData_;

            static constexpr int extension_level = 3;
        }; // class Map_Utility
    } // namespace AA_SIPP
} // namespace Low_Level_Engine
#pragma once

#include <queue>

#include "server/MAPF_Util.hpp"

using namespace MAPF_Util;

namespace Instance
{
    namespace AgentInstance
    {
        struct Agent
        {
            std::string name_;

            double size_;
            double wheel_radius_;
            double wheel_seperation_;

            double max_linVel_, max_linAcc_;
            double max_angVel_, max_angAcc_;

            Position::Pose start_;
            Position::Pose goal_;
            Position::Pose pose_;

            friend std::ostream &operator<<(std::ostream &_os, const Agent &_agent)
            {
                _os << "Agent Info"             << std::endl;
                _os << "- Name: "               << _agent.name_             << std::endl;
                _os << "- Size: "               << _agent.size_             << std::endl;
                _os << "- Wheel Radius: "       << _agent.wheel_radius_     << std::endl;
                _os << "- Wheel Seperation: "   << _agent.wheel_seperation_ << std::endl;
                _os << "- Start: "              << _agent.start_            << std::endl;
                _os << "- Goal: "               << _agent.goal_             << std::endl;
                _os << "- Current Pose: "       << _agent.pose_             << std::endl;
                
                _os << "- Maximum linear velocity     : "   << _agent.max_linVel_   << std::endl;
                _os << "- Maximum linear acceleration : "   << _agent.max_linAcc_   << std::endl;
                _os << "- Maximum angular acceleration: "   << _agent.max_angVel_   << std::endl;
                _os << "- Maximum angular acceleration: "   << _agent.max_angAcc_   << std::endl;

                return _os;
            }

            Agent() {} 
        }; // struct Agent
    } // namespace AgnetInstance

    namespace MapInstance
    {
        struct Cell
        {
        public:
            Position::Index idx_;
            Position::Coordinates coord_;
            bool occupied_;

        private:
            double distance_;
            Position::Coordinates obstacle_coord_;
            friend class BinaryOccupancyMap;
            
        public:
            Cell &operator=(const Cell &_other)
            {
                coord_          = _other.coord_;
                idx_            = _other.idx_;
                occupied_       = _other.occupied_;
                obstacle_coord_ = _other.obstacle_coord_;

                return *this;
            }

            friend std::ostream &operator<<(std::ostream &_os, const Cell &_cell)
            {
                _os << _cell.coord_ << ": " << _cell.occupied_;

                return _os;
            }

            // For priority queue
            friend bool operator<(const Cell &_first, const Cell &_second)
            {
                return _first.distance_ > _second.distance_;
            }
        
        public:
            Cell(const Cell& _other)
            {
                coord_          = _other.coord_;
                idx_            = _other.idx_;
                occupied_       = _other.occupied_;
                obstacle_coord_ = _other.obstacle_coord_;
            }
        }; // struct Cell

        class BinaryOccupancyMap
        {
        public:
            struct MapProperty
            {
                Position::Coordinates origin_;
                int width_, height_;
                double resolution_;
                double inflation_radius_;

                MapProperty &operator=(const MapProperty &_other)
                {
                    this->origin_           = _other.origin_;
                    this->width_            = _other.width_;
                    this->height_           = _other.height_;
                    this->resolution_       = _other.resolution_;
                    this->inflation_radius_ = _other.inflation_radius_;

                    return *this;
                }

                friend std::ostream &operator<<(std::ostream &_os, const MapProperty &_mapProperty)
                {
                    _os << "- Map Property"     << std::endl;
                    _os << "  o Origin      : " << _mapProperty.origin_     << std::endl;
                    _os << "  o Width       : " << _mapProperty.width_      << std::endl;
                    _os << "  o Height      : " << _mapProperty.height_     << std::endl;
                    _os << "  o Resolution  : " << _mapProperty.resolution_ << std::endl;

                    return _os;
                }

                MapProperty(const MapProperty &_other)
                {
                    this->origin_           = _other.origin_;
                    this->width_            = _other.width_;
                    this->height_           = _other.height_;
                    this->resolution_       = _other.resolution_;
                    this->inflation_radius_ = _other.inflation_radius_;
                }

                MapProperty()
                {
                    this->origin_           = Position::Coordinates();
                    this->width_            = std::numeric_limits<int>::quiet_NaN();
                    this->height_           = std::numeric_limits<int>::quiet_NaN();
                    this->resolution_       = std::numeric_limits<double>::quiet_NaN();
                    this->inflation_radius_ = std::numeric_limits<double>::quiet_NaN();
                }
            }; // struct MapProperty

        public:
            BinaryOccupancyMap &operator=(const BinaryOccupancyMap &_other);
            std::vector<Position::Index> getInflatedArea(const std::vector<Position::Index> &_rootArea,
                                                         const double &_inflation_radius);
            std::vector<std::vector<Cell>> inflate(const double &_inflation_radius);
            bool isOutofMap(const Cell &_cell) const;
        
        private:
            const double distanceLookup(const Cell &_cell) const;
            void enqueue(const Instance::MapInstance::Cell &_cell, const double &_inflation_radius);
        
        public:
            MapProperty property_;
            std::vector<std::vector<Cell>> mapData_;
            std::vector<std::vector<Cell>> inflated_mapData_;
        
        private:
            std::vector<std::vector<bool>> seen_;
            std::priority_queue<Cell> inflation_queue_;
        
        public:
            BinaryOccupancyMap() {}
        }; // class BinaryOccupancyMap

        const double getDistance(const Cell &_first, const Cell &_second)
        {
            return Position::getDistance(_first.coord_, _second.coord_);
        }
    } // namespace MapInstance
} // namespace Instance
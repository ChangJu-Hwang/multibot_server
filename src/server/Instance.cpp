#include "server/Instance.hpp"

using namespace Instance;

MapInstance::BinaryOccupancyMap &MapInstance::BinaryOccupancyMap::operator=(const MapInstance::BinaryOccupancyMap &_other)
{
    property_           = _other.property_;
    mapData_            = _other.mapData_;
    inflated_mapData_   = _other.inflated_mapData_;

    return *this;
}

std::vector<Position::Index> MapInstance::BinaryOccupancyMap::getInflatedArea(const std::vector<Position::Index> &_rootArea,
                                                                              const double &_inflation_radius)
{
    try
    {
        if (std::isnan(_inflation_radius) or _inflation_radius < 0)
            throw _inflation_radius;
    }
    catch(const double &_invalid_inflation_radius)
    {
        std::cerr << "[Error] BinaryOccupancyMap::getInflatedArea(): "
                  << "Invalid Inflation Radius: " << _invalid_inflation_radius << std::endl;
        std::abort();
    }

    seen_.clear();
    seen_.resize(property_.width_, std::vector<bool>(property_.height_, false));
    while (not(inflation_queue_.empty()))
        inflation_queue_.pop();

    for (const auto &idx : _rootArea)
    {
        MapInstance::Cell rootCell  = mapData_[idx.x_][idx.y_];
        rootCell.obstacle_coord_    = mapData_[idx.x_][idx.y_].coord_;

        enqueue(rootCell, _inflation_radius);
    }

    std::vector<Position::Index> inflatedArea;
    inflatedArea.clear();
    while (not(inflation_queue_.empty()))
    {
        const MapInstance::Cell &current_cell   = inflation_queue_.top();
        inflatedArea.push_back(current_cell.idx_);

        if (current_cell.idx_.x_ > 0)
        {
            MapInstance::Cell neighbor_cell = current_cell;
            neighbor_cell.idx_.x_   = current_cell.idx_.x_ - 1;
            neighbor_cell.coord_.x_ = property_.origin_.x_ + neighbor_cell.idx_.x_ * property_.resolution_;
            neighbor_cell.coord_.y_ = property_.origin_.y_ + neighbor_cell.idx_.y_ * property_.resolution_;
            enqueue(neighbor_cell, _inflation_radius);
        }
        if (current_cell.idx_.y_ > 0)
        {
            MapInstance::Cell neighbor_cell = current_cell;
            neighbor_cell.idx_.y_   = current_cell.idx_.y_ - 1;
            neighbor_cell.coord_.x_ = property_.origin_.x_ + neighbor_cell.idx_.x_ * property_.resolution_;
            neighbor_cell.coord_.y_ = property_.origin_.y_ + neighbor_cell.idx_.y_ * property_.resolution_;
            enqueue(neighbor_cell, _inflation_radius);
        }
        if(current_cell.idx_.x_ < property_.width_ - 1)
        {
            MapInstance::Cell neighbor_cell = current_cell;
            neighbor_cell.idx_.x_   = current_cell.idx_.x_ + 1;
            neighbor_cell.coord_.x_ = property_.origin_.x_ + neighbor_cell.idx_.x_ * property_.resolution_;
            neighbor_cell.coord_.y_ = property_.origin_.y_ + neighbor_cell.idx_.y_ * property_.resolution_;
            enqueue(neighbor_cell, _inflation_radius);
        }
        if(current_cell.idx_.y_ < property_.height_ - 1)
        {
            MapInstance::Cell neighbor_cell = current_cell;
            neighbor_cell.idx_.y_   = current_cell.idx_.y_ + 1;
            neighbor_cell.coord_.x_ = property_.origin_.x_ + neighbor_cell.idx_.x_ * property_.resolution_;
            neighbor_cell.coord_.y_ = property_.origin_.y_ + neighbor_cell.idx_.y_ * property_.resolution_;
            enqueue(neighbor_cell, _inflation_radius);
        }

        inflation_queue_.pop();
    }

    return inflatedArea;
}

std::vector<std::vector<MapInstance::Cell>> MapInstance::BinaryOccupancyMap::inflate(const double &_inflation_radius)
{
    property_.inflation_radius_ = _inflation_radius;
    inflated_mapData_           = mapData_;

    std::vector<Position::Index> occupiedCell_Indexes;
    occupiedCell_Indexes.clear();
    for (const auto &row : mapData_)
    {
        for (const auto &cell : row)
        {
            if (cell.occupied_)
                occupiedCell_Indexes.push_back(cell.idx_);
        }
    }
    occupiedCell_Indexes    = getInflatedArea(occupiedCell_Indexes, _inflation_radius);

    for(const auto &idx : occupiedCell_Indexes)
        inflated_mapData_[idx.x_][idx.y_].occupied_ = true;
    
    return inflated_mapData_;
}

bool MapInstance::BinaryOccupancyMap::isOutofMap(const MapInstance::Cell &_cell) const
{
    return not(_cell.idx_.x_ >= 0 and _cell.idx_.x_ < property_.width_ and
               _cell.idx_.y_ >= 0 and _cell.idx_.y_ < property_.height_);
}

const double MapInstance::BinaryOccupancyMap::distanceLookup(const MapInstance::Cell &_cell) const
{
    return (Position::getDistance(_cell.coord_, _cell.obstacle_coord_));
}

void MapInstance::BinaryOccupancyMap::enqueue(const MapInstance::Cell &_cell, const double &_inflation_ratdius)
{
    if (not(seen_[_cell.idx_.x_][_cell.idx_.y_] and
        not(distanceLookup(_cell) > _inflation_ratdius + 1e-8)))
    {
        seen_[_cell.idx_.x_][_cell.idx_.y_] = true;

        MapInstance::Cell newCell = _cell;

        newCell.distance_       = distanceLookup(_cell);
        newCell.obstacle_coord_ = _cell.obstacle_coord_;
        inflation_queue_.push(newCell);
    }
}
#include "multibot_server/AA_SIPP_Map_Utility.hpp"

using namespace Low_Level_Engine;

void AA_SIPP::Map_Utility::restrictArea(
    const std::pair<Position::Coordinates, Position::Coordinates> &_searchSpace)
{
    if (_searchSpace.first.x_ != -1 * std::numeric_limits<double>::infinity() and
        _searchSpace.first.y_ != -1 * std::numeric_limits<double>::infinity())
    {
        Position::Index input_lower_left;
        input_lower_left.x_ = std::round((_searchSpace.first.x_ - space_margine_ - map_.property_.origin_.x_) / map_.property_.resolution_);
        input_lower_left.y_ = std::round((_searchSpace.first.y_ - space_margine_ - map_.property_.origin_.y_) / map_.property_.resolution_);

        lower_left_.x_ = std::max(input_lower_left.x_, lower_left_.x_);
        lower_left_.y_ = std::max(input_lower_left.y_, lower_left_.y_);
    }

    if (_searchSpace.second.x_ != std::numeric_limits<double>::infinity() and
        _searchSpace.second.y_ != std::numeric_limits<double>::infinity())
    {
        Position::Index input_upper_right;
        input_upper_right.x_ = std::round((_searchSpace.second.x_ + space_margine_ - map_.property_.origin_.x_) / map_.property_.resolution_);
        input_upper_right.y_ = std::round((_searchSpace.second.y_ + space_margine_ - map_.property_.origin_.y_) / map_.property_.resolution_);

        upper_right_.x_ = std::min(input_upper_right.x_, upper_right_.x_);
        upper_right_.y_ = std::min(input_upper_right.y_, upper_right_.y_);
    }
}

std::vector<Position::Index> AA_SIPP::Map_Utility::getNeighborIndex(const Position::Index &_index) const
{
    std::vector<std::pair<int, int>> neighborIdx_candidates(0);
    {
        neighborIdx_candidates.push_back({1, 0});
        neighborIdx_candidates.push_back({0, 1});
        neighborIdx_candidates.push_back({-1, 0});
        neighborIdx_candidates.push_back({0, -1});
    }
    if (extension_level_ > 2)
    {
        neighborIdx_candidates.push_back({1, 1});
        neighborIdx_candidates.push_back({-1, 1});
        neighborIdx_candidates.push_back({-1, -1});
        neighborIdx_candidates.push_back({1, -1});
    }
    if (extension_level_ > 3)
    {
        neighborIdx_candidates.push_back({2, 1});
        neighborIdx_candidates.push_back({2, 1});
        neighborIdx_candidates.push_back({-1, 2});
        neighborIdx_candidates.push_back({-2, 1});
        neighborIdx_candidates.push_back({-2, -1});
        neighborIdx_candidates.push_back({-1, -2});
        neighborIdx_candidates.push_back({1, -2});
        neighborIdx_candidates.push_back({2, -1});
    }
    if (extension_level_ > 4)
    {
        neighborIdx_candidates.push_back({3, 1});
        neighborIdx_candidates.push_back({3, 2});
        neighborIdx_candidates.push_back({2, 3});
        neighborIdx_candidates.push_back({1, 3});
        neighborIdx_candidates.push_back({-1, 3});
        neighborIdx_candidates.push_back({-2, 3});
        neighborIdx_candidates.push_back({-3, 2});
        neighborIdx_candidates.push_back({-3, 1});
        neighborIdx_candidates.push_back({-3, -2});
        neighborIdx_candidates.push_back({-3, -1});
        neighborIdx_candidates.push_back({-2, -3});
        neighborIdx_candidates.push_back({-1, -3});
        neighborIdx_candidates.push_back({1, -3});
        neighborIdx_candidates.push_back({3, -2});
        neighborIdx_candidates.push_back({3, -2});
        neighborIdx_candidates.push_back({3, -1});
    }

    std::vector<Position::Index> neighbors;
    neighbors.clear();
    for (const auto &relative_index : neighborIdx_candidates)
    {
        Position::Index neighbor_idx(
            _index.x_ + relative_index.first,
            _index.y_ + relative_index.second);

        if (neighbor_idx.x_ < lower_left_.x_ or
            neighbor_idx.y_ < lower_left_.y_)
            continue;

        if (neighbor_idx.x_ > upper_right_.x_ or
            neighbor_idx.y_ > upper_right_.y_)
            continue;

        neighbors.push_back(neighbor_idx);
    }

    return neighbors;
}

std::vector<Position::Index> AA_SIPP::Map_Utility::getNeighborIndex(const Position::Pose &_pose) const
{
    Position::Index index = convertPoseToIndex(_pose);

    return getNeighborIndex(index);
}

std::vector<Position::Index> AA_SIPP::Map_Utility::getValidIndexes(
    const std::string &_agentName, const std::vector<Position::Index> &_target)
{
    double agentSize = agentSizeDB_[_agentName] + std::sqrt(2) * map_.property_.resolution_;

    if (std::isnan(map_.property_.inflation_radius_) or
        std::fabs(map_.property_.inflation_radius_ - agentSize) > 1e-8)
    {
        double inflation_radius = agentSize;
        inflated_mapData_ = map_.inflate(inflation_radius);
    }

    std::vector<Position::Index> validIndexes;
    validIndexes.clear();
    for (const auto &index : _target)
    {
        if (inflated_mapData_[index.x_][index.y_].occupied_)
            continue;

        validIndexes.push_back(index);
    }

    return validIndexes;
}

bool AA_SIPP::Map_Utility::isValidIndexes(
    const std::string &_agentName, const std::vector<Position::Index> &_target)
{
    double agentSize = agentSizeDB_[_agentName];

    if (std::isnan(map_.property_.inflation_radius_) or
        std::fabs(map_.property_.inflation_radius_ - agentSize) > 1e-8)
    {
        double inflation_radius = agentSize;
        inflated_mapData_ = map_.inflate(inflation_radius);
    }

    for (const auto &index : _target)
    {
        if (inflated_mapData_[index.x_][index.y_].occupied_)
            return false;
    }

    return true;
}

const Position::Pose AA_SIPP::Map_Utility::convertIndexToPose(
    const Position::Index &_target,
    const Position::Pose &_from, const Position::Pose &_to) const
{
    double x = map_.property_.origin_.x_ + _target.x_ * map_.property_.resolution_;
    double y = map_.property_.origin_.y_ + _target.y_ * map_.property_.resolution_;
    double theta = _to.component_.theta;

    if (x == _from.component_.x and
        y == _from.component_.y)
        theta = _from.component_.theta;

    return Position::Pose(x, y, theta);
}

const Position::Pose AA_SIPP::Map_Utility::convertIndexToPose(
    const Position::Index &_target,
    double _theta) const
{
    double x = map_.property_.origin_.x_ + _target.x_ * map_.property_.resolution_;
    double y = map_.property_.origin_.y_ + _target.y_ * map_.property_.resolution_;
    
    return Position::Pose(x, y, _theta);
}

const Position::Index AA_SIPP::Map_Utility::convertPoseToIndex(const Position::Pose &_target) const
{
    int x = std::round((_target.component_.x - map_.property_.origin_.x_) / map_.property_.resolution_);
    int y = std::round((_target.component_.y - map_.property_.origin_.y_) / map_.property_.resolution_);

    return Position::Index(x, y);
}

const std::vector<Position::Index> AA_SIPP::Map_Utility::getRouteComponents(
    const Position::Pose &_from, const Position::Pose &_to) const
{
    Position::Index from = convertPoseToIndex(_from);
    Position::Index to = convertPoseToIndex(_to);

    int x0 = from.x_, y0 = from.y_;
    int x1 = to.x_, y1 = to.y_;

    bool steep = false;
    if (std::abs(y1 - y0) > std::abs(x1 - x0))
    {
        steep = true;
        std::swap(x0, y0);
        std::swap(x1, y1);
    }

    int deltaX = std::abs(x1 - x0), deltaY = std::abs(y1 - y0);
    int error = 0;
    int deltaError = deltaY;

    int x = x0, y = y0;

    int xStep = x0 < x1 ? 1 : -1;
    int yStep = y0 < y1 ? 1 : -1;

    std::vector<Position::Index> routeComponents;
    routeComponents.clear();
    while (x != x1 + xStep)
    {
        Position::Index componentIndex;
        if (steep)
        {
            if (not(map_.isOutofMap(map_.mapData_[y][x])))
            {
                componentIndex.x_ = y;
                componentIndex.y_ = x;
            }
        }
        else
        {
            if (not(map_.isOutofMap(map_.mapData_[x][y])))
            {
                componentIndex.x_ = x;
                componentIndex.y_ = y;
            }
        }
        routeComponents.push_back(componentIndex);

        x = x + xStep;
        error = error + deltaError;

        if (2 * error >= deltaX)
        {
            y = y + yStep;
            error = error - deltaX;
        }
    }

    return routeComponents;
}
#include "server/MAPF_Util.hpp"

using namespace MAPF_Util;

double Position::getDistance(const Position::Pose &_first,
                                   const Position::Pose &_second)
{
    double deltaX = _first.component_.x - _second.component_.x;
    double deltaY = _first.component_.y - _second.component_.y;

    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

double Position::getAngleDiff(const Position::Pose &_first,
                                    const Position::Pose &_second)
{
    double angleDiff = std::fabs(_first.component_.theta - _second.component_.theta);
    while(angleDiff - M_PI > 1e-8)
        angleDiff = std::fabs(angleDiff - 2 * M_PI);
    
    return angleDiff;
}
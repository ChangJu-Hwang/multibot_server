#include "multibot_server/AA_SIPP_Motion.hpp"

using namespace Low_Level_Engine;

const Time::TimePoint AA_SIPP::Motion::getMoveTime(
    const std::string &_agentName,
    const Position::Pose &_from, const Position::Pose &_to) const
{
    return getPartialMoveTime(_agentName, _to, _from, _to);
}

const Time::TimePoint AA_SIPP::Motion::getPartialMoveTime(
    const std::string &_agentName, const Position::Pose &_target,
    const Position::Pose &_from, const Position::Pose &_to) const
{
    return Time::TimePoint();
}

const Time::TimePoint AA_SIPP::Motion::getPartialMoveTime(
    const std::string &_agentName, const Position::Index &_target,
    const Position::Index &_from, const Position::Index &_to) const
{
    return Time::TimePoint();
}

const Time::TimePoint AA_SIPP::Motion::MoveTimeComputer(
    const double _s,
    const double _max_s, const double _max_v, const double _max_a) const
{
    return Time::TimePoint();
}
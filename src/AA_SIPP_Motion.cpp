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
    auto agent = agents_.find(_agentName)->second;

    double max_linVel = agent.max_linVel_;
    double max_linAcc = agent.max_linAcc_;
    double max_angVel = agent.max_angVel_;
    double max_angAcc = agent.max_angAcc_;


    double angle = Position::getAngleDiff(_from, _target);
    double max_angle = Position::getAngleDiff(_from, _to);
    Time::TimePoint time_to_rotate = MoveTimeComputer(
        angle,
        max_angle, max_angVel, max_angAcc);

    double distance = Position::getDistance(_from, _target);
    double max_distance = Position::getDistance(_from, _to);
    Time::TimePoint time_to_traverse = MoveTimeComputer(
        distance,
        max_distance, max_linVel, max_linAcc);

    return Time::TimePoint(time_to_rotate + time_to_traverse);
}

const Time::TimePoint AA_SIPP::Motion::getPartialMoveTime(
    const std::string &_agentName, const Position::Index &_target,
    const Position::Pose &_from, const Position::Pose &_to) const
{
    Position::Pose targetPose = map_utility_->convertIndexToPose(
        _target, _from, _to);
    
    return getPartialMoveTime(_agentName, targetPose, _from, _to);
}

const Time::TimePoint AA_SIPP::Motion::MoveTimeComputer(
    const double _s,
    const double _max_s, const double _max_v, const double _max_a) const
{
    try
    {
        if (_s > _max_s + 1e-8 or _s < 0)
        {
            std::cout << _max_s << std::endl;
            throw _s;
        }
    }
    catch (const double _wrong_s)
    {
        std::cerr << "[Error] AA_SIPP_Motion::MoveTimeComputer(): "
                  << "Invalid displacement " << _wrong_s << std::endl;
        std::abort();
    }

    if (_s < 1e-8)
        return Time::TimePoint(0);

    double timePointValue;

    // Trapezoidal Velocity Profile
    if (_max_s > _max_v * _max_v / _max_a + 1e-8)
    {
        if (_max_v * _max_v / (2 * _max_a) > _s + 1e-8)
            timePointValue = std::sqrt(2 * _s / _max_a);
        else if (_max_s - _max_v * _max_v / (2 * _max_a) > _s + 1e-8)
            timePointValue = _s / _max_v + _max_v / (2 * _max_a);
        else
            timePointValue = _max_s / _max_v + _max_v / _max_a - std::sqrt(2 * std::fabs(_max_s - _s) / _max_a);
    }
    // Triangular Velocity Profile
    else
    {
        if (_max_s / 2 > _s + 1e-8)
            timePointValue = std::sqrt(2 * _s / _max_a);
        else
            timePointValue = std::sqrt(4 * _max_s / _max_a) - std::sqrt(2 * std::fabs(_max_s - _s) / _max_a);
    }

    return Time::TimePoint(timePointValue);
}
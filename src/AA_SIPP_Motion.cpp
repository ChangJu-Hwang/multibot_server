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

const Position::Pose AA_SIPP::Motion::getPosition(
    const std::string &_agentName, const Time::TimePoint &_key, const Time::TimePoint &_start_time,
    const Position::Pose &_from, const Position::Pose &_to)
{
    if (_start_time + std::numeric_limits<Time::TimePoint>::epsilon() > _key )
        return _from;

    double max_angle    = Position::getAngleDiff(_from, _to);

    double max_angVel = agents_[_agentName].max_angVel_;
    double max_angAcc = agents_[_agentName].max_angAcc_;
    Time::TimePoint rotation_duration = MoveTimeComputer(
        max_angle,
        max_angle, max_angVel, max_angAcc);
    
    Position::Pose pose = _from;
    if (_key - _start_time < rotation_duration + std::numeric_limits<Time::TimePoint>::epsilon())
    {
        Position::Coordinates start_unitVector(cos(_from.component_.theta), sin(_from.component_.theta));
        Position::Coordinates goal_unitVector(cos(_to.component_.theta), sin(_to.component_.theta));

        int sign = (Position::crossProduct(start_unitVector, goal_unitVector) > 0) ? 1 : -1;

        pose.component_.theta = pose.component_.theta + sign * DisplacementComputer(
            _key - _start_time,
            max_angle, max_angVel, max_angAcc); 
    }
    else
    {
        pose.component_.theta = _to.component_.theta;

        double displacement = DisplacementComputer(
            _key - _start_time - rotation_duration,
            Position::getDistance(_from, _to), agents_[_agentName].max_linVel_, agents_[_agentName].max_linAcc_);
        
        pose.component_.x = pose.component_.x + displacement * cos(pose.component_.theta);
        pose.component_.y = pose.component_.y + displacement * sin(pose.component_.theta);
    }

    return pose;
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

double AA_SIPP::Motion::DisplacementComputer(
    const Time::TimePoint &_key,
    const double _max_s, const double _max_v, const double _max_a) const
{
    if (_max_s < 1e-8)
        return 0.0;
    
    // Trapezoidal Velocity Profile
    if (_max_s > _max_v * _max_v / _max_a + 1e-8)
    {
        if (_key.count() < _max_v / _max_a)
            return (0.5 * _max_a * _key.count() * _key.count());
        else if (_key.count() < _max_s / _max_v)
            return (_max_v * _key.count() - 0.5 * _max_v * _max_v / _max_a);
        else
            return (_max_s - 0.5 * _max_a * std::pow(_max_s / _max_v + _max_v / _max_a - _key.count(),2));
    }
    // Triangular Velocity Profile
    else
    {
        if (_key.count() < std::sqrt(_max_s / _max_a))
            return (0.5 * _max_a * _key.count() * _key.count());
        else
            return (_max_s - 0.5 * _max_a * std::pow(2 * std::sqrt(_max_s / _max_a) - _key.count(),2));
    }
}
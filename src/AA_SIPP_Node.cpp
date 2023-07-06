#include "multibot_server/AA_SIPP_Node.hpp"

using namespace Low_Level_Engine;

void AA_SIPP::Node::updateHeuristic(
    const Position::Pose &_goal,
    const double _max_linVel)
{
    hVal_ = Position::getDistance(pose_, _goal) / _max_linVel;
}

bool AA_SIPP::Node::validationCheck() const
{
    if (pose_.component_.theta + 1e-8 < -M_PI or
        pose_.component_.theta + 1e-8 > M_PI)
        return false;

    if (not(safe_interval_.is_safe_))
        return false;

    if (safe_interval_.startTime_.count() + 1e-8 > safe_interval_.endTime_.count())
        return false;

    if (arrival_time_.count() > departure_time_.count() + 1e-8)
        return false;

    if (safe_interval_.startTime_.count() > arrival_time_.count() + 1e-8)
        return false;

    if (departure_time_.count() + 1e-8 > safe_interval_.endTime_.count())
        return false;

    return true;
}

AA_SIPP::Node &AA_SIPP::Node::operator=(const AA_SIPP::Node &_other)
{
    idx_ = _other.idx_;
    pose_ = _other.pose_;

    safe_interval_ = _other.safe_interval_;
    arrival_time_ = _other.arrival_time_;
    departure_time_ = _other.departure_time_;

    gVal_ = _other.gVal_;
    hVal_ = _other.hVal_;
    parent_ = _other.parent_;

    return *this;
}

bool AA_SIPP::Node::operator==(const AA_SIPP::Node &_other) const
{
    if (not(idx_ == _other.idx_))
        return false;

    if (not(safe_interval_ == _other.safe_interval_))
        return false;

    return true;
}

bool AA_SIPP::Node::operator!=(const AA_SIPP::Node &_other) const
{
    return not(*this == _other);
}

AA_SIPP::Node::Node(const AA_SIPP::Node &_other)
{
    idx_ = _other.idx_;
    pose_ = _other.pose_;

    safe_interval_ = _other.safe_interval_;
    arrival_time_ = _other.arrival_time_;
    departure_time_ = _other.departure_time_;

    gVal_ = _other.gVal_;
    hVal_ = _other.hVal_;
    parent_ = _other.parent_;
}
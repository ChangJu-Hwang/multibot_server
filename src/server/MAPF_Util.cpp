#include "server/MAPF_Util.hpp"

using namespace MAPF_Util;

Position::Coordinates &Position::Coordinates::operator=(const Position::Coordinates &_other)
{
    this->x_    = _other.x_;
    this->y_    = _other.y_;

    return *this;
}

Position::Coordinates &Position::Coordinates::operator=(const std::vector<double> &_other)
{
    this->x_    = _other[CARTESIAN::x];
    this->y_    = _other[CARTESIAN::y];

    return *this;
}

Position::Coordinates &Position::Coordinates::operator=(const std::pair<double, double> &_other)
{
    this->x_    = _other.first;
    this->y_    = _other.second;    

    return *this;
}

bool Position::Coordinates::operator==(const Position::Coordinates &_other) const
{
    if(std::fabs(this->x_ - _other.x_) > 1e-8)
        return false;
    
    if(std::fabs(this->y_ - _other.y_) > 1e-8)
        return false;
    
    return true;
}

bool Position::Coordinates::operator!=(const Position::Coordinates &_other) const
{
    return not(*this == _other);
}

const Position::Coordinates Position::Coordinates::operator+(const Position::Coordinates &_other) const
{
    Position::Coordinates result    = *this;
        result.x_   = this->x_ + _other.x_;
        result.y_   = this->y_ + _other.y_;

    return result;
}

const Position::Coordinates Position::Coordinates::operator-(const Position::Coordinates &_other) const
{
    Position::Coordinates result    = *this;
        result.x_   = this->x_ - _other.x_;
        result.y_   = this->y_ - _other.y_;

    return result;
}

const Position::Coordinates Position::Coordinates::operator*(const double &_coefficient) const
{
    Position::Coordinates result    = *this;
        result.x_   = this->x_ * _coefficient;
        result.y_   = this->y_ * _coefficient;
    
    return result;
}

double Position::Coordinates::operator*(const Position::Coordinates &_other) const
{
    return (this->x_ * _other.x_ + this->y_ * _other.y_);
}

const Position::Coordinates Position::Coordinates::operator/(const double &_divider) const
{
    Position::Coordinates result = *this;

    try
    {
        if(std::fabs(_divider) < 1e-8)
            throw std::runtime_error("Math Error: Attemped to divide by Zero\n");

        result.x_   = this->x_ / _divider;
        result.y_   = this->y_ / _divider;
    }
    catch(const std::runtime_error& e)
    {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::abort();
    }

    return result;
}

double Position::Coordinates::norm()
{
    return std::sqrt(x_ * x_ + y_ * y_);
}

double Position::getDistance(const Position::Coordinates &_first,
                             const Position::Coordinates &_second)
{
    double deltaX   = _first.x_ - _second.x_;
    double deltaY   = _first.y_ - _second.y_;

    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

double Position::crossProduct(const Position::Coordinates &_first,
                              const Position::Coordinates &_second)
{
    return _first.x_ * _second.y_ - _first.y_ * _second.x_;
}

double Position::getDistance(const Position::Pose &_first,
                             const Position::Pose &_second)
{
    double deltaX   = _first.component_.x - _second.component_.x;
    double deltaY   = _first.component_.y - _second.component_.y;

    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

double Position::getAngleDiff(const Position::Pose &_first,
                                    const Position::Pose &_second)
{
    double angleDiff    = std::fabs(_first.component_.theta - _second.component_.theta);
    while(angleDiff - M_PI > 1e-8)
        angleDiff   = std::fabs(angleDiff - 2 * M_PI);
    
    return angleDiff;
}
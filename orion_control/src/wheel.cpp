#include <cmath>
#include "orion_control/wheel.hpp"

namespace orion_control
{
    void Wheel::SetUp(const std::string& wheel_name, int tickers_per_rev)
    {
        this->name_ = wheel_name;
        this->rads_per_tick = (2 * M_PI) / tickers_per_rev;
    }

    double Wheel::Angle()
    {
        return this->enc_ * this->rads_per_tick_;
    }
}

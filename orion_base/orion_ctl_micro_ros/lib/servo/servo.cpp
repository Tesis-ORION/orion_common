#include <cmath>

#include "servo.hpp"

namespace fwd
{
    void ServoMotor::begin()
    {
        this->servo_.attach(this->pwm_pin_);
    }

    void ServoMotor::setPositionDeg(const float& degrees)
    {
        if(degrees > 180)
        {
            this->servo_.write(180);
        }
        else if (degrees < 0)
        {
            this->servo_.write(0);
        }
        else
        {
            this->servo_.write(degrees);  
        }
    }

    float ServoMotor::getPositionDeg()
    {
        return this->servo_.read();
    }

    void ServoMotor::setPositionRad(const float& radians)
    {
        float pos = radians * M_PI / 180;
        if(pos > 180)
        {
            this->servo_.write(180);
        }
        else if (pos < 0)
        {
            this->servo_.write(0);
        }
        else
        {
            this->servo_.write(pos);  
        }
    }

    float ServoMotor::getPositionRad()
    {
        return (this->servo_.read() * M_PI / 180);
    }
}
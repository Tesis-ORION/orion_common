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
        if(degrees > this->max_pos_)
        {
            this->servo_.write(this->max_pos_);
        }
        else if (degrees < this->min_pos_)
        {
            this->servo_.write(this->min_pos_);
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
        float pos = (float) (radians / M_PI * 180.0);
        if(pos > this->max_pos_)
        {
            this->servo_.write(this->max_pos_);
        }
        else if (pos < this->min_pos_)
        {
            this->servo_.write(this->min_pos_);
        }
        else
        {
            this->servo_.write(pos);  
        }
    }

    float ServoMotor::getPositionRad()
    {
        return ( (float) this->servo_.read() * M_PI / 180.0);
    }

    void ServoMotor::approximatePositionDeg()
    {
        const int pos = this->getPositionDeg();
        const int diff =  pos - this->objective_;
        if(!(abs(diff) <= 1.0))
        {
            if(diff > 0)
            {
                this->setPositionDeg(pos + (diff > 10 ? 10 : diff));
            }
            else
            {
                this->setPositionDeg(pos - (diff < -10 ? 10 : diff));
            }
        }
    }

    void ServoMotor::setObjectiveDeg(float degrees)
    {
        this->objective_ = degrees;
    }

    void ServoMotor::setObjectiveRad(float radians)
    {
        this->setObjectiveDeg(radians * 180 / M_PI);
    }
}
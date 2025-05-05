#ifndef SERVO_HPP
#define SERVO_HPP

#include <ESP32Servo.h>

namespace fwd
{
    class ServoMotor
    {
    private:
        unsigned int max_pos_{180};
        unsigned int min_pos_{0};
        unsigned int pwm_pin_{0};
        float position_{0};
        float objective_{0};
        Servo servo_;
    public:
        ServoMotor(const unsigned int max_pos, const unsigned int min_pos, 
            const unsigned int pwm_pin)
            : max_pos_{max_pos}, min_pos_{min_pos}, pwm_pin_{pwm_pin} {}

        void begin();
        void setPositionDeg(const float& degrees);
        float getPositionDeg();
        void setPositionRad(const float& radians);
        float getPositionRad();
        void approximatePositionDeg();
        void setObjectiveDeg(float degrees);
        void setObjectiveRad(float radians);
    };
}

#endif
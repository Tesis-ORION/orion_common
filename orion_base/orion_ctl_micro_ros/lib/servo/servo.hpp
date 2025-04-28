#ifndef SERVO_HPP
#define SERVO_HPP

#include <ESP32Servo.h>

namespace fwd
{
    class ServoMotor
    {
    private:
        static constexpr int MAX_POS{180};
        static constexpr int MIN_POS{0};
        unsigned int pwm_pin_{0};
        float position_{0};
        Servo servo_;
    public:
        ServoMotor(const unsigned pwm_pin) : pwm_pin_{pwm_pin} {}

        void begin();
        void setPositionDeg(const float& degrees);
        float getPositionDeg();
        void setPositionRad(const float& radians);
        float getPositionRad();
    };
}

#endif
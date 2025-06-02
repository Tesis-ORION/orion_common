// //////////////////////// Include Libraries //////////////////////////////
// ------------------------- Custom dependencies ---------------------------
#include "pid.hpp" // Custom PID class header

// ////////////////////////// CLASS DEFINITIONS ////////////////////////////
namespace diff
{
    /**
     * Calculate the PID output based on the error (input - reference), the
     * constante defined and considering the clamping to avoid saturation.
     * 
     * @param enc_count Current value of the encoder count of the motor
     * @param computed_output Pointer to the PWM output
     */
    void ControlPID::compute(int enc_count, int& computed_output)
    {
        // Check that the PID is disabled
        if(!this->enabled_)
        {
            // If so, take advantage to reset the PID.
            if(this->last_input_ != 0)
            {
                reset(enc_count);
            }
            return;
        }

        // Set up input
        int input = enc_count - this->last_enc_count_;

        // Determinate the error
        long err = this->setpoint_ - input;

        // Calculate output
        // Considering PID with a Ko value to avoid too much increment
        long output = (this->kp_ * err - this->kd_ * (input - this->last_input_)
            + this->integral_term_) / this->ko_;
        
        // Sum previous output
        output += this->last_output_;

        // Clamp to avoid saturation
        if(output > this->pwm_max_)
        {
            output = this->pwm_max_;
        }
        else if (output < this->pwm_min_)
        {
            output = this->pwm_min_;
        }
        else
        {
            // If there is no saturation, add the integral term for next iter.
            this->integral_term_ += this->ki_ * err;
        }

        // Update value of the commanded output
        computed_output = output;

        // Prepare values for the next iteration
        this->last_enc_count_ = enc_count;
        this->last_input_ = input;
        this->last_output_ = output;
    
    } // void ControlPID::compute()
        
    /**
     * Disable the PID.
     */
    void ControlPID::disable()
    {
        this->enabled_ = false;

    } // void ControlPID::disabled()
    
    /**
     * Enamble the PID
     */
    void ControlPID::enable()
    {
        this->enabled_ = true;

    } // void ControlPID::enable()
    
    /**
     * Check that the PID is enabled or not
     * 
     * @return True if it is enabled, false otherwise.
     */
    bool ControlPID::enabled()
    {
        return this->enabled_;

    } // bool ControlPID::enabled()
    
    /**
     * Reset the PID values for the computation.
     * 
     * @param enc_count Latest encoder count
     */
    void ControlPID::reset(int enc_count)
    {
        this->setpoint_ = 0;
        this->integral_term_ = 0;
        this->last_enc_count_ = enc_count;
        this->last_input_ = 0;
        this->last_output_ = 0;

    } // void ControlPID::reset()

    /**
     * Update set point (objective) of the controller
     * 
     * @param setpoint Encoder count to achieve
     */
    void ControlPID::setSetpoint(int setpoint)
    {
        this->setpoint_ = setpoint;

    } // void ControlPID::setSetpoint()

    /**
     * Update the constants of the PID
     * 
     * @param kp Proportional constant
     * @param kd Derivative constant
     * @param ki Integral constant
     * @param ko Saturation constant
     */
    void ControlPID::setTunings(int kp, int kd, int ki, int ko)
    {
        this->kp_ = kp;
        this->kd_ = kd;
        this->ki_ = ki;
        this->ko_ = ko;

    } // void ControlPID::setTunings

} // namespace diff

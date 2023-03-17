/**
 * Thrust Vectoring PID Controller
 **/

class PID {
public:
    // Constructor to initialize variables
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f) 
        : kp_(kp), ki_(ki), kd_(kd), last_input_(0), output_(0) {
        set_output_limits(-400.0f, 400.0f);
    }

    // Function to update the PID controller
    float update_pid_std(float setpoint, float input, float dt) {
        // Calculate the error
        float error = setpoint - input;

        // Calculate the integral error term
        sum_err_ += error * ki_ * dt;

        // Calculate the derivative error term
        float derivative = kd_ / dt * (input - last_input_);
        last_input_ = input;

        // Calculate the output
        output_ = kp_ * error + sum_err_ + derivative;

        // Check the output against the output limits
        if (output_ > outmax_) {
            sum_err_ = (outmax_ - kp_ * error - derivative) / ki_;
            output_ = outmax_;
        }
        else if (output_ < outmin_) {
            sum_err_ = (outmin_ - kp_ * error - derivative) / ki_;
            output_ = outmin_;
        }

        // Return the output
        return output_;
    }

    // Function to reset the PID controller
    void reset() {
        sum_err_ = 0.0f;
        last_input_ = 0.0f;
    }

    // Function to set kp, ki, and kd
    void set_Kpid(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    // Function to set the output limits to prevent integral windup
    void set_output_limits(float min, float max) {
        outmin_ = min;
        outmax_ = max;
    }

private:
    float kp_;          // Proportional gain
    float ki_;          // Integral gain
    float kd_;          // Derivative gain
    float last_input_;  // Last input
    float output_;      // Output
    float sum_err_ = 0; // Sum of errors, initialized to zero
    float outmax_;      // Maximum output limit
    float outmin_;      // Minimum output limit
};

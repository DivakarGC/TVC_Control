/**
 * Thrust Vectoring PID Controller
 **/

// Define the PID class
class PID {
public:
    // Constructor to initialize variables to zero
    PID() {
        kp = 0;
        ki = 0;
        kd = 0;
        err = 0;
        sum_err = 0;
        dif_err = 0;
        lastInput = 0;
        outmax = 400;
        outmin = -400;
    }

    // Constructor to allow user to set kp, ki, and kd
    PID(float kp_, float ki_, float kd_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;

        err = 0;
        sum_err = 0;
        dif_err = 0;
        lastInput = 0;
        outmax = 400;
        outmin = -400;
    }

    // Function to update the PID controller
    float update_pid_std(float setpoint, float input, float dt) {
        // Calculate the error
        err = setpoint - input;

        // Calculate the integral error term
        sum_err += err * ki * dt;

        // Calculate the derivative error term
        dif_err = -kd / dt * (input - lastInput);

        // Calculate the output
        output = kp * err + sum_err + dif_err;

        // Check the output against the output limits
        if (output > outmax) {
            sum_err = 0.0;
            output = outmax;
        }
        else if (output < outmin) {
            sum_err = 0.0;
            output = outmin;
        }

        // Store the current input as the last input
        lastInput = input;

        // Return the output
        return output;
    }

    // Function to reset the PID controller
    void reset() {
        sum_err = 0;
        dif_err = 0;
        lastInput = 0;
    }

    // Function to set kp, ki, and kd
    void set_Kpid(float KP, float KI, float KD) {
        kp = KP;
        ki = KI;
        kd = KD;
    }

    // Function to set the output limits to prevent integral windup
    void set_windup_bounds(float min, float max) {
        outmax = max;
        outmin = min;
    }

private:
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float kd;        // Derivative gain
    float err;       // Current error
    float sum_err;   // Sum of errors
    float dif_err;   // Difference of errors
    float lastInput; // Last input
    float output;    // Output
    float outmax;    // Maximum output limit
    float outmin;    // Minimum output limit
};


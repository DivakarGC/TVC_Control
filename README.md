# Thrust Vectoring PID Controller

Welcome to the Thrust Vectoring PID Controller, a humble yet optimized implementation of a PID controller in C++! This code is designed to help maintain the desired orientation of a thrust vector control system, using a modular and customizable PID controller.

## Features

1. Modular and customizable design
2. Optimized for speed and efficiency
3. Easy-to-use functions for setting PID gains and windup bounds
4. Built-in function for resetting the system

## Usage

The PID class can be easily integrated into any C++ project, simply by including the PID.h header file and creating an instance of the PID class. The controller can be updated using the update_pid_std() function, which takes in the setpoint, input, and time delta as arguments.

The controller gains can be set using the set_Kpid() function, and the output limits can be set using the set_windup_bounds() function to prevent integral windup.

## Example

Here's an example of how to use the PID controller:

```cpp #include "PID.h"

// Create a new instance of the PID controller
PID controller(1.0, 0.5, 0.2);

// Set the output limits to prevent integral windup
controller.set_windup_bounds(-100, 100);

// Update the controller every 0.1 seconds
float setpoint = 10.0;
float input = 8.0;
float dt = 0.1;
float output = controller.update_pid_std(setpoint, input, dt);
```
## Contributions

Contributions to this project are always welcome! Please feel free to submit a pull request with any optimizations or improvements you may have.

## Thank you :)

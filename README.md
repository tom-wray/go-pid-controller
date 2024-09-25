# Go PID Controller

This repository contains a simple and efficient implementation of a Proportional-Integral-Derivative (PID) controller in Go. PID controllers are widely used in control systems for various applications, including robotics, industrial processes, and more.

## Features

- Proportional, Integral, and Derivative control
- Configurable gains (Kp, Ki, Kd)
- Output limiting (Min/Max)
- Deadband for ignoring small errors
- Anti-windup protection
- Saturation detection

## Installation

To install this PID controller in your Go project, follow these steps:

```
go get github.com/yourusername/go-pid-controller
```

## Configuration

The PID controller can be configured with the following parameters:

- `Kp`: Proportional gain
- `Ki`: Integral gain
- `Kd`: Derivative gain
- `MinOutput`: Minimum output value
- `MaxOutput`: Maximum output value
- `Deadband`: Error values smaller than this are treated as zero
- `AntiWindup`: Enable/disable anti-windup protection

## Example

Here's a simple example of how to use the PID controller:

```
package main

import (
    "fmt"
    "time"
    "github.com/tom-wray/go-pid-controller"
)

func main() {
    pid := pidcontroller.New(1.0, 0.1, 0.05, -100, 100)
    pid := PID{
        Kp: 1.0
        Ki: 0.1
        Kd: 0.05
        MinOutput: -100
        MaxOutput: 100
        Deadband: 0.1
        AntiWindup: true
    }
    setpoint := 50.0
    measuredValue := 0.0
    for i := 0; i < 100; i++ {
        error := setpoint - measuredValue
        output := pid.Calculate(error)
        // Simulate system response (replace with your actual system)
        measuredValue += output * 0.1
        fmt.Printf("Iteration %d: Error = %.2f, Output = %.2f, Measured Value = %.2f\n",
        i, error, output, measuredValue)
        time.Sleep(100 * time.Millisecond)
    }
}
```

You can also view the test file for different implementations of the controller.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

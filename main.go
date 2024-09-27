package pid

import (
	"math"
	"time"
)

// PID represents a Proportional-Integral-Derivative controller.
type PID struct {
	// parameters
	Kp         float64 // Proportional gain
	Ki         float64 // Integral gain
	Kd         float64 // Derivative gain
	MinOutput  float64 // Minimum output value
	MaxOutput  float64 // Maximum output value
	Deadband   float64 // Deadband to ignore small errors
	Saturated  bool    // Indicates if the output is saturated
	AntiWindup bool    // Enable/disable anti-windup protection

	// internal state
	prevError  float64   // Previous error for derivative calculation
	integral   float64   // Integral sum
	lastTime   time.Time // Last update time
	lastOutput float64   // Last output value
}

// Update calculates and returns the control output based on the current reference value.
//
// Parameters:
//   - setpoint: The desired setpoint
//   - measured: The current measured value of the process variable
//
// Returns:
//   - float64: The calculated control output
func (pid *PID) Update(setpoint, measured float64) float64 {
	now := time.Now()
	if pid.lastTime.IsZero() {
		pid.lastTime = now
	}

	dt := time.Since(pid.lastTime).Seconds()
	if dt <= 0 {
		return pid.lastOutput
	}

	error := setpoint - measured

	// Apply deadband
	if math.Abs(error) < pid.Deadband {
		return pid.lastOutput
	}

	// calculate P term
	pTerm := pid.Kp * error

	// Calculate D term (only if Kd is non-zero)
	dTerm := 0.0
	if pid.Kd != 0 {
		dTerm = pid.Kd * (error - pid.prevError) / dt
	}

	// Calculate I term (only if Ki is non-zero)
	iTerm := 0.0
	integral := 0.5 * dt * (error + pid.prevError)
	if pid.Ki != 0 {
		iTerm = pid.Ki * pid.integral
	}

	output := pTerm + iTerm + dTerm

	// Apply output limits
	saturated := false
	if output > pid.MaxOutput {
		output = pid.MaxOutput
		saturated = error > 0
	} else if output < pid.MinOutput {
		output = pid.MinOutput
		saturated = error < 0
	}

	// update previous error, time, and last output for use in next iteration
	pid.prevError = error
	pid.lastTime = now
	pid.lastOutput = output
	pid.Saturated = saturated
	if pid.AntiWindup && !pid.Saturated {
		pid.integral = integral
	}

	return output
}

// Reset resets the PID controller's internal state.
// This includes resetting the previous error, integral sum, and last update time.
func (pid *PID) Reset() {
	pid.prevError = 0
	pid.integral = 0
	pid.lastTime = time.Now()
}

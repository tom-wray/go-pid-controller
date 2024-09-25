package pid

import (
	"math"
	"testing"
	"time"
)

func TestPIDUpdate(t *testing.T) {
	tests := []struct {
		name     string
		pid      PID
		setpoint float64
		measured float64
		expected float64
	}{
		{
			name: "Basic P controller",
			pid: PID{
				Kp: 1.0, Ki: 0, Kd: 0,
				MinOutput: -100, MaxOutput: 100,
			},
			setpoint: 10,
			measured: 5,
			expected: 5, // (setpoint - measured) * Kp
		},
		{
			name: "P controller with limits",
			pid: PID{
				Kp: 10.0, Ki: 0, Kd: 0,
				MinOutput: -20, MaxOutput: 20,
			},
			setpoint: 10,
			measured: 5,
			expected: 20, // Limited to MaxOutput
		},
		{
			name: "PID controller",
			pid: PID{
				Kp: 1.0, Ki: 0.1, Kd: 0.05,
				MinOutput: -100, MaxOutput: 100,
			},
			setpoint: 10,
			measured: 5,
			expected: 5.05, // Approximate, as it depends on time
		},
		{
			name: "Controller with deadband",
			pid: PID{
				Kp: 1.0, Ki: 0, Kd: 0,
				MinOutput: -100, MaxOutput: 100,
				Deadband: 2,
			},
			setpoint: 10,
			measured: 9,
			expected: 0, // Within deadband, so no change
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			output := 0.0
			for i := 0; i < 100; i++ {
				output = tt.pid.Update(tt.setpoint, tt.measured)
				time.Sleep(10 * time.Millisecond)
			}
			if math.Abs(output-tt.expected) > 0.1 {
				t.Errorf("Expected output close to %v, but got %v", tt.expected, output)
			}
		})
	}
}

func TestPIDReset(t *testing.T) {
	pid := PID{
		Kp: 1.0, Ki: 0.1, Kd: 0.05,
		MinOutput: -100, MaxOutput: 100,
	}

	// Update the PID to set some internal state
	for i := 0; i < 20; i++ {
		pid.Update(10, 5)
		time.Sleep(10 * time.Millisecond)
	}

	// Reset the PID
	pid.Reset()

	if pid.prevError != 0 {
		t.Errorf("Expected prevError to be 0 after reset, but got %v", pid.prevError)
	}

	if pid.integral != 0 {
		t.Errorf("Expected integral to be 0 after reset, but got %v", pid.integral)
	}

	if time.Since(pid.lastTime) > time.Second {
		t.Errorf("Expected lastTime to be recent after reset, but got %v", pid.lastTime)
	}
}

func TestPIDAntiWindup(t *testing.T) {
	pid := PID{
		Kp: 1.0, Ki: 0.1, Kd: 0,
		MinOutput: -10, MaxOutput: 10,
		AntiWindup: true,
	}

	// Run multiple updates to accumulate integral term
	for i := 0; i < 100; i++ {
		pid.Update(100, 0) // Large error to force saturation
	}

	// Check if integral term is limited
	if math.Abs(pid.integral) > pid.MaxOutput/pid.Ki {
		t.Errorf("Expected integral to be limited, but got %v", pid.integral)
	}
}

func TestPIDSaturation(t *testing.T) {
	pid := PID{
		Kp: 10.0, Ki: 0, Kd: 0,
		MinOutput: -20, MaxOutput: 20,
	}

	output := 0.0
	for i := 0; i < 100; i++ {
		output = pid.Update(10, 0)
		time.Sleep(10 * time.Millisecond)
	}

	if output != pid.MaxOutput {
		t.Errorf("Expected output to be saturated at MaxOutput (%v), but got %v", pid.MaxOutput, output)
	}

	if !pid.Saturated {
		t.Errorf("Expected Saturated flag to be true, but it was false")
	}
}

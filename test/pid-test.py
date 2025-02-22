import math
import matplotlib.pyplot as plt

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class PID:
    def __init__(self, kp, ki, kd, rate, cutoff_freq, i_limit, o_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.rate = rate
        self.dt = 1.0 / rate
        self.i_limit = i_limit
        self.o_limit = o_limit
        self.last_error = 0
        self.integral = 0
        self.filtered_derivative = 0
        self.tau = 1 / (2 * math.pi * cutoff_freq) if cutoff_freq > 0 else 0

    def update(self, error):
        output = self.kp * error

        raw_derivative = (error - self.last_error) * self.rate
        self.last_error = error

        if self.tau > 0:
            alpha = self.dt / (self.tau + self.dt)
            self.filtered_derivative = (self.filtered_derivative * (1.0 - alpha) +
                                        raw_derivative * alpha)
        else:
            self.filtered_derivative = raw_derivative

        output += self.kd * self.filtered_derivative

        self.integral += error * self.dt
        if self.i_limit > 0:
            self.integral = clamp(self.integral, -self.i_limit, self.i_limit)
        
        output += self.ki * self.integral

        if self.o_limit > 0:
            output = clamp(output, -self.o_limit, self.o_limit)
        
        return output

    def reset(self):
        self.last_error = 0
        self.integral = 0

# Simple simulator for drone height control with two PID loops
def simulate_pid():
    outer_pid = PID(kp=6.0, ki=1.0, kd=0.5, rate=100, cutoff_freq=15, i_limit=2, o_limit=4)
    inner_pid = PID(kp=5.0, ki=5.0, kd=5.0, rate=100, cutoff_freq=30, i_limit=40, o_limit=200)
    
    target_height = 1.0  # Desired height in meters
    height = 0.0  # Initial height
    velocity = 0.0  # Initial velocity
    dt = 1.0 / 100  # Simulation step time
    gravity = -9.81  # Gravity effect
    thrust_to_acceleration = 0.001  # Conversion factor for thrust to acceleration

    time_steps = 1000  # Number of simulation steps
    heights = []
    times = []
    velocitys = []
    target_velocitys = []
    
    for t in range(time_steps):
        height_error = target_height - height
        target_velocity = outer_pid.update(height_error)  # Outer loop: height control
        target_velocitys.append(target_velocity)
        
        velocity_error = target_velocity - velocity
        thrust = inner_pid.update(velocity_error) * 100  # Inner loop: velocity control
        
        acceleration = thrust * thrust_to_acceleration
        velocity += acceleration * dt
        velocitys.append(velocity)

        height += velocity * dt
        
        heights.append(height)
        times.append(t * dt)
    
    plt.plot(times, heights, label='Height')
    plt.axhline(target_height, color='r', linestyle='--', label='Target Height')
    plt.plot(times, velocitys, label='Velocity')
    plt.plot(times, target_velocitys, label='Target Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Height (m)')
    plt.title('PID Control of Drone Hovering Height (Two Loops)')
    plt.legend()
    plt.show()

# Run simulation
simulate_pid()
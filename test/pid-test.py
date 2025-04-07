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
    outer_pid = PID(kp=5.0, ki=0.6, kd=1.0, rate=100, cutoff_freq=20, i_limit=0, o_limit=1.2)
    inner_pid = PID(kp=50.0, ki=2.0, kd=0.3, rate=100, cutoff_freq=40, i_limit=20, o_limit=250)
    
    # outer_pid = PID(kp=10.0, ki=4.0, kd=0.0, rate=500, cutoff_freq=40, i_limit=20, o_limit=0)
    # inner_pid = PID(kp=60.0, ki=20.0, kd=3.0, rate=500, cutoff_freq=100, i_limit=50, o_limit=0)
    
    target_height = 1.5  # Desired height in meters
    height = 0.0  # Initial height
    velocity = 0.0  # Initial velocity
    dt = 1.0 / 100  # Simulation step time
    gravity = -9.81  # Gravity effect
    thrust_to_acceleration = 0.0083  # Conversion factor for thrust to acceleration

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
        
        acceleration = thrust * thrust_to_acceleration + gravity  # Calculate acceleration
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

def simulate_rpy_pid():
    outer_pid = PID(kp=10.0, ki=4.0, kd=0.0, rate=500, cutoff_freq=40, i_limit=20, o_limit=0)
    inner_pid = PID(kp=250.0, ki=200.0, kd=3.0, rate=500, cutoff_freq=100, i_limit=50, o_limit=0)
    
    target_roll = 30.0  # Desired roll in degrees
    roll = 0.0  # Initial roll
    roll_speed = 0.0  # Initial roll speed
    dt = 1.0 / 100  # Simulation step time
    gravity = -9.81  # Gravity effect
    thrust_to_acceleration = 0.083  # Conversion factor for thrust to acceleration

    time_steps = 1000  # Number of simulation steps
    rolls = []
    times = []
    roll_speeds = []
    target_roll_speeds = []
    
    for t in range(time_steps):
        roll_error = target_roll - roll
        target_roll_speed = outer_pid.update(roll_error)  # Outer loop: roll control
        target_roll_speeds.append(target_roll_speed)
        
        roll_speed_error = target_roll_speed - roll_speed
        thrust = inner_pid.update(roll_speed_error)  # Inner loop: velocity control
        
        acceleration = thrust * thrust_to_acceleration
        roll_speed += acceleration * dt
        roll_speeds.append(roll_speed)

        roll += roll_speed * dt
        
        rolls.append(roll)
        times.append(t * dt)
    
    plt.plot(times, rolls, label='rolls')
    plt.axhline(target_roll, color='r', linestyle='--', label='Target Roll')
    # plt.plot(times, roll_speeds, label='Velocity')
    # plt.plot(times, target_roll_speeds, label='Target Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Roll (degrees)')
    plt.title('PID Control of Drone Hovering Roll (Two Loops)')
    plt.legend()
    plt.show()


# Run simulation
simulate_rpy_pid()
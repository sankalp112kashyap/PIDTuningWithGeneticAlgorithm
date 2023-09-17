import numpy as np
import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 1.0

    def update(self, current_position):
        error = self.setpoint - current_position
        proportional_term = self.kp * error
        self.integral += self.ki * (error * self.dt)
        derivative_term = self.kd * ((error - self.prev_error)/self.dt)
        control_signal = proportional_term + self.integral + derivative_term
        self.prev_error = error
        return control_signal

class MotorSimulator:
    def __init__(self, inertia, friction, initial_position=0.0):
        self.inertia = inertia
        self.friction = friction
        self.angular_position = initial_position
        self.angular_velocity = 0.0
        self.dt = 1.0

    def update(self, control_signal):
        # Simulate motor dynamics (Euler's method)
        acceleration = control_signal - self.friction * self.angular_velocity
        self.angular_velocity += acceleration / self.inertia * self.dt
        self.angular_position += self.angular_velocity * self.dt

        return self.angular_position

# Define PID gains to be optimized
kp = 0.5                   
ki = 0.2
kd = 0.1

 
# kp = 0.46987385
# ki = 0.0 
# kd = 0.86472198


# Initialize the PID controller and motor simulator
setpoint = np.pi / 3.0  # Desired angular position (45 degrees)
current_position = 0.0  # Initial position (starting value)

pid_controller = PIDController(kp, ki, kd, setpoint)
motor_simulator = MotorSimulator(inertia=1.0, friction=0.1, initial_position=current_position)

# Simulate the motor control loop
time_steps = 100
total_error = 0.0

# Lists to store data for plotting
time_points = []
current_position_values = []

for t in range(time_steps):
    print(current_position)

    # Store data for plotting
    time_points.append(t)
    current_position_values.append(current_position)

    # current_angular position
    current_position = motor_simulator.update(pid_controller.update(motor_simulator.angular_position))

    error = setpoint - current_position
    total_error += abs(error)

    # Pause for a sec (dt) moment (simulating real-time control)
    time.sleep(1)


# Calculate and print the total error as a performance measure
print("Total Error:", total_error)


# Plotting the results
import matplotlib.pyplot as plt

plt.plot(time_points, current_position_values)
plt.xlabel('Time (s)')
plt.ylabel('Current position (in degrees)')
plt.title('PID Control of Angular Posistion of Satellite Motor simulation')
plt.grid(True)
plt.show()

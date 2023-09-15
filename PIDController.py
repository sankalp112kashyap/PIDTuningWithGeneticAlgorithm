import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, current_position):
        error = self.setpoint - current_position
        proportional_term = self.kp * error
        self.integral += self.ki * error
        derivative_term = self.kd * (error - self.prev_error)
        control_signal = proportional_term + self.integral + derivative_term
        self.prev_error = error
        return control_signal

class MotorSimulator:
    def __init__(self, inertia, friction, initial_position=0.0):
        self.inertia = inertia
        self.friction = friction
        self.angular_position = initial_position
        self.angular_velocity = 0.0

    def update(self, control_signal):
        # Simulate motor dynamics (Euler's method)
        acceleration = control_signal - self.friction * self.angular_velocity
        self.angular_velocity += acceleration / self.inertia
        self.angular_position += self.angular_velocity

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


for _ in range(time_steps):
    print(current_position)

    current_position = motor_simulator.update(pid_controller.update(motor_simulator.angular_position))

    error = setpoint - current_position
    total_error += abs(error)


# print ("For 10 generation with exact (fine tuned with genetic algorithm) K gains")
# Calculate and print the total error as a performance measure
print("Total Error:", total_error)

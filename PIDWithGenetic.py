import numpy as np

# Define the PID controller class
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

# Define the motor simulator class
class MotorSimulator:
    def __init__(self, inertia, friction):
        self.inertia = inertia
        self.friction = friction
        self.angular_position = 0.0
        self.angular_velocity = 0.0

    def update(self, control_signal):
        # Simulate motor dynamics (Euler's method)
        acceleration = control_signal - self.friction * self.angular_velocity
        self.angular_velocity += acceleration / self.inertia
        self.angular_position += self.angular_velocity

        return self.angular_position

# Define the fitness function to evaluate a set of PID gains
def fitness_function(gains):
    setpoint = np.pi / 3.0  # Desired angular position (45 degrees)

    pid_controller = PIDController(*gains, setpoint)
    motor_simulator = MotorSimulator(inertia=1.0, friction=0.1)

    time_steps = 100
    total_error = 0.0

    for _ in range(time_steps):
        current_position = motor_simulator.update(pid_controller.update(motor_simulator.angular_position))
        error = setpoint - current_position
        total_error += abs(error)

    # Calculate fitness (minimize the total error)
    fitness = -total_error

    return fitness

# Define genetic algorithm parameters
population_size = 50
num_generations = 200
mutation_rate = 0.1

# Define the range for PID gains
gain_min = 0
gain_max = 1.0

# Initialize the population with random PID gains
population = [np.random.uniform(gain_min, gain_max, 3) for _ in range(population_size)]

# Genetic Algorithm Loop
for generation in range(num_generations):
    fitness_scores = [fitness_function(individual) for individual in population]

    # Select the top-performing individuals (elite selection)
    elite_indices = np.argsort(fitness_scores)[-int(population_size * 0.2):]
    elite_population = [population[i] for i in elite_indices]

    new_population = elite_population[:]

    while len(new_population) < population_size:
        parent1 = elite_population[np.random.randint(len(elite_population))]
        parent2 = elite_population[np.random.randint(len(elite_population))]

        # Crossover (average of parents)
        child = (parent1 + parent2) / 2.0

        # Mutation
        if np.random.rand() < mutation_rate:
            mutation_amount = np.random.uniform(-0.1, 0.1, len(child))
            child += mutation_amount
            child = np.clip(child, gain_min, gain_max)

        new_population.append(child)

    population = new_population

# Select the best individual from the final population
best_individual_index = np.argmax([fitness_function(individual) for individual in population])
best_gains = population[best_individual_index]

print("Best PID Gains:", best_gains)
print("Best Fitness:", fitness_function(best_gains))
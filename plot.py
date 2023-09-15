import matplotlib.pyplot as plt

# Read data from three files (replace 'file1.txt', 'file2.txt', 'file3.txt' with your file names)
def read_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    values = [float(line.strip()) for line in lines[:21]]
    return values

# give appropriate file names below from data needs to be read 
data1 = read_data('60_10.txt')
data2 = read_data('60_20.txt')
data3 = read_data('60_200.txt')
data4 = read_data('60_NonTuned.txt')

# Generate time values assuming a 1-second interval between each entry
time_interval = 1  # 1 second interval
time_values = [i * time_interval for i in range(len(data1))]  # Assuming all files have the same length

# Create a line chart with three lines (one for each file)
plt.figure(figsize=(10, 6))
plt.plot(time_values, data1, label='Tuned PDI - 10 Generations', marker='o', linestyle='-', color='b')
plt.plot(time_values, data2, label='Tuned PDI - 20 Generations', marker='x', linestyle='--', color='g')
plt.plot(time_values, data3, label='Tuned PDI - 200 Generations', marker='s', linestyle=':', color='r')
plt.plot(time_values, data4, label='Untuned PDI', marker='^', linestyle='-.', color='m')


plt.xlabel('Time (seconds)')
plt.ylabel('Current Angular Position (radians)')
plt.title('Current Angular Position for Untuned and Tuned PDI ( with Genetic Algorithm for 200, 20, 10 number of generations)')
plt.grid(True)
plt.legend()


plt.show()

import serial
# import matplotlib
import csv
import numpy as np
# matplotlib.use('Agg')  # Set the backend before importing pyplot
import matplotlib.pyplot as plt

# Open the serial port (adjust the parameters as needed)
ser = serial.Serial(
    port='/dev/ttyACM1',       # Windows: COMx, Linux/Mac: /dev/ttyUSB0 or /dev/ttyACM0
    baudrate=115200,     # Common rates: 9600, 19200, 38400, 57600, 115200
    bytesize=serial.EIGHTBITS,  # Data bits (5-8)
    parity=serial.PARITY_NONE,  # Parity: None, Even, Odd
    stopbits=serial.STOPBITS_ONE,  # Stop bits: 1, 1.5, 2
    timeout=1          # Read timeout (seconds)
)

TITLE_TYPE = "SS_10Hz"

# def scatter(x, y):
#     plt.scatter(x, y)
def scatter(x, y):
    plt.figure()  # Create new figure
    plt.scatter(x, y, s=0.7)  # s=1 makes the dots very small
    # plt.ylim(0.3, 0.7)  # Adjust these values based on your expected distance range
    plt.xlabel('time_steps')
    plt.ylabel('Distance (meters)')
    plt.title('Distance vs time_steps')
    plt.grid(True)
    plt.savefig("distance_scatter_" + TITLE_TYPE + "_anchor.png")

    # plt.scatter(x, y, s=0.1, linewidths=0)  # s=1 makes the dots very small
    # plt.ylabel('Distance (meters)')
    # plt.title('Distance vs time_steps')
    # plt.grid(True)
    # plt.savefig('distance_scatter_1_anchor.png')

# def boxPlot(x, y):
#     plt.boxplot(y)
def boxPlot(_, y):  # Use _ for unused parameter
    plt.figure(figsize=(10, 20))  # Create larger figure with width=10, height=20
    plt.boxplot(y)
    plt.ylim(min(y) - 0.1, max(y) + 0.1)  # Adjust y-axis limits
    plt.yticks(np.arange(min(y) - 0.1, max(y) + 0.1, 0.01))  # Set y-axis ticks every 0.01 units
    plt.title('Distance vs time_steps')
    plt.grid(True)
    plt.savefig('distance_boxplot_' + TITLE_TYPE + '_anchor.png', bbox_inches='tight')  # tight layout to use full space

def histogram(_, y):
    plt.figure(figsize=(10, 20))  # Create larger figure
    min_val, max_val = min(y), max(y)
    # Create evenly spaced bins with a small buffer
    bins = np.linspace(min_val - 0.001, max_val + 0.001, len(set(y)) + 1)
    
    plt.hist(y, bins=bins, linewidth=1, edgecolor='black', orientation='horizontal')
    plt.yticks(sorted(set(y)))  # Show all distinct values on y-axis
    plt.xlabel('Frequency')
    plt.ylabel('Distance (meters)')
    plt.title('Distance Histogram')
    plt.grid(True, axis='y')  # Only show horizontal grid lines
    plt.savefig('distance_histogram_' + TITLE_TYPE + '_anchor.png', bbox_inches='tight')
# Use _ for unused parameter

TIME = 2500

x = [] # timestep
y = [] # distance
avgDistance = 0
time = 0
## Read data from the serial port (example)
while time < TIME:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Read and decode data
        processed_data = data.split()
        
        if len(processed_data) >= 2:
            time += 1
            avgDistance += round(float(processed_data[1]), 2)  # Round each input to 2 decimal places
            # print(processed_data, time)
            
            if time % 10 == 0:
                curTime = time / 10.0
                avgDistance = round(avgDistance / 10.0, 2)  # Round the average to 2 decimal places
                print(f"{avgDistance:.2f}", curTime)  # Format with exactly 2 decimal places
                x.append(curTime)
                y.append(avgDistance)
                avgDistance = 0
            # x.append(time)
            # y.append(float(processed_data[1]))

boxPlot(x, y)
scatter(x, y)
histogram(x, y)

with open('distance_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['timestep', 'distance'])  # header
    writer.writerows(zip(x, y))  # data
# Close the connection when done

ser.close()

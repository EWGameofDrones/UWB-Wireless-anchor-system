import serial
import matplotlib
import csv
import numpy as np
matplotlib.use('Agg')  # Set the backend before importing pyplot
import time
import matplotlib.pyplot as plt


import sys

# Open the serial port (adjust the parameters as needed)
ser = serial.Serial(
    port='/dev/ttyACM3',       # Windows: COMx, Linux/Mac: /dev/ttyUSB0 or /dev/ttyACM0
    baudrate=115200,     # Common rates: 9600, 19200, 38400, 57600, 115200
    bytesize=serial.EIGHTBITS,  # Data bits (5-8)
    parity=serial.PARITY_NONE,  # Parity: None, Even, Odd
    stopbits=serial.STOPBITS_ONE,  # Stop bits: 1, 1.5, 2
    timeout=1          # Read timeout (seconds)
)

A, B, C = 0, 0, 0

def trilateration(d1, d2, d3):
    """
    Performs 3D trilateration to find both possible intersection points of three spheres.
    Anchors are kept in x-y plane (z=0) but returns 3D positions for the responder.
    :param d1: Distance from anchor (0, 0, 0)
    :param d2: Distance from anchor (83, 150, 0)
    :param d3: Distance from anchor (38, 260, 0)
    :return: Formatted string with position in meters
    """
    # Static anchor positions (all z=0)
    x1, y1, z1 = 0, 0, 0
    x2, y2, z2 = 83 * 0.0254, 150 * 0.0254, 0
    x3, y3, z3 = 38 * 0.0254, 260 * 0.0254, 0
    
    # Find circle intersections in x-y plane
    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    if d > d1 + d2 or d < abs(d1 - d2):
        return None  # Circles don't intersect
    
    # Find intersection points of first two circles
    a = (d1**2 - d2**2 + d**2)/(2*d)
    h = np.sqrt(d1**2 - a**2)
    
    x_base = x1 + (a/d)*(x2-x1)
    y_base = y1 + (a/d)*(y2-y1)
    
    x3_1 = x_base + (h/d)*(y2-y1)
    y3_1 = y_base - (h/d)*(x2-x1)
    x3_2 = x_base - (h/d)*(y2-y1)
    y3_2 = y_base + (h/d)*(x2-x1)
    
    dist1 = np.sqrt((x3_1-x3)**2 + (y3_1-y3)**2)
    dist2 = np.sqrt((x3_2-x3)**2 + (y3_2-y3)**2)
    
    solutions = []
    eps = 0.01
    
    # Find the solution that's closest to d3
    z1 = np.sqrt(max(0, d3**2 - dist1**2))  # Use max to avoid negative square roots
    z2 = np.sqrt(max(0, d3**2 - dist2**2))
    
    pos1 = [x3_1, y3_1, z1]
    pos2 = [x3_2, y3_2, z2]
    
    # Compare distances to third anchor to determine best solution
    calc_dist1 = np.sqrt((x3_1-x3)**2 + (y3_1-y3)**2 + z1**2)
    calc_dist2 = np.sqrt((x3_2-x3)**2 + (y3_2-y3)**2 + z2**2)
    
    # Choose the solution that's closest to the measured distance d3
    if abs(calc_dist1 - d3) <= abs(calc_dist2 - d3):
        return f"Position: X={pos1[0]:.2f}m, Y={pos1[1]:.2f}m, Z={pos1[2]:.2f}m"
    else:
        return f"Position: X={pos2[0]:.2f}m, Y={pos2[1]:.2f}m, Z={pos2[2]:.2f}m"


start_time = time.time()

totalMsgs = 0
while start_time + 20 > time.time():
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Read and decode data
        if time.time() - start_time < 2:
            continue
        processed_data = data.split()
        # print(processed_data)
        if float(processed_data[1]) <= 0:
            continue

        if processed_data[0] == 'A':
            A = float(processed_data[1])
        elif processed_data[0] == 'B':
            B = float(processed_data[1])
        elif processed_data[0] == 'C':
            C = float(processed_data[1])

        if A != 0 and B != 0 and C != 0:
            # print(A, B, C)
            pos = trilateration(A, B, C)
            if pos is not None:
                print(pos) 
                totalMsgs += 1
            # else:
            #     print("No unique solution")

            A, B, C = 0, 0, 0

print("Total messages: ", totalMsgs)
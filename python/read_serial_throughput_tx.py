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
    port='/dev/ttyACM1',       # Windows: COMx, Linux/Mac: /dev/ttyUSB0 or /dev/ttyACM0
    baudrate=115200,     # Common rates: 9600, 19200, 38400, 57600, 115200
    bytesize=serial.EIGHTBITS,  # Data bits (5-8)
    parity=serial.PARITY_NONE,  # Parity: None, Even, Odd
    stopbits=serial.STOPBITS_ONE,  # Stop bits: 1, 1.5, 2
    timeout=1          # Read timeout (seconds)
)

if len(sys.argv) != 3:
    print(sys.argv)
    exit() 

index = sys.argv[1]
delay = sys.argv[2]

TITLE_TYPE = "ss_twr: throughput " + str(delay) + " ms delay, distance ~35 feet, " + str(index)


def bar(msgs_Received, msgs_Dropped):
    plt.figure()
    x = [0, 1]
    heights = [msgs_Received, msgs_Dropped]
    plt.bar(x, heights, color=['g', 'r'], label=['Successful rangings', 'Failed rangings'])
    for i, h in enumerate(heights):
        plt.text(x[i], h, str(h), ha='center', va='bottom')
    plt.xticks([])
    plt.ylabel('Messages')
    plt.title(TITLE_TYPE)
    plt.legend()
    plt.savefig(TITLE_TYPE + '.png')
    plt.close()

start_time = time.time()
duration = 20  # 20 seconds

msgs_Received = 0
msgs_Dropped = 0

prev_val = 0
first = True

totalMessages = 0

# Read data from the serial port (example) 
# NOTE TO SELF, simplify type casting by explicitly storing processed_data[2] as an int
while (time.time() - start_time) < duration:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Read and decode data
        processed_data = data.split()
        # print(processed_data)

        if len(processed_data) >= 2:
            totalMessages += 1
            if totalMessages < 100: # To clear UART buffer
                continue
            try:
                current_val = int(processed_data[1])
                msgs_Received += 1
                if (current_val != prev_val + 1 and not first):
                    print(prev_val, current_val)
                    msgs_Dropped += current_val - prev_val
                prev_val = current_val
                first = False
            except ValueError:
                msgs_Dropped += 1
                # print("msgs_Dropped data format at elapsed:", time.time() - start_time, " data:", processed_data)
bar(msgs_Received, msgs_Dropped)
print("Total messages tx: ", totalMessages)

ser.close()

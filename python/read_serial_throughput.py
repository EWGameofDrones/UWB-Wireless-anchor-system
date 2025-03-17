import sys
import serial
import matplotlib
import csv
import numpy as np
matplotlib.use('Agg')  # Set the backend before importing pyplot
import time
import matplotlib.pyplot as plt
import math

# Open the serial port (adjust the parameters as needed)
ser = serial.Serial(
    port='/dev/ttyACM0',       # Windows: COMx, Linux/Mac: /dev/ttyUSB0 or /dev/ttyACM0
    baudrate=115200,     # Common rates: 9600, 19200, 38400, 57600, 115200
    bytesize=serial.EIGHTBITS,  # Data bits (5-8)
    parity=serial.PARITY_NONE,  # Parity: None, Even, Odd
    stopbits=serial.STOPBITS_ONE,  # Stop bits: 1, 1.5, 2
    timeout=1          # Read timeout (seconds)
)

if len(sys.argv) > 2:
    print(sys.argv)
    exit() 

index = sys.argv[1]

TITLE_TYPE = "DBL_BUFF_INT RX: throughput 1ms delay, ~35feet distance, " + index


def bar(msgs_Received, msgs_Dropped):
    plt.figure()
    x = [0, 1]
    heights = [msgs_Received, msgs_Dropped]
    plt.bar(x, heights, color=['g', 'r'], label=['Messages received', 'Messages dropped'])
    for i, h in enumerate(heights):
        plt.text(x[i], h, str(h), ha='center', va='bottom')
    plt.xticks([])
    plt.ylabel('Messages')
    plt.title(TITLE_TYPE)
    plt.legend()
    plt.savefig(TITLE_TYPE)
    plt.close()

start_time = time.time()
duration = 20  # 20 seconds

msgs_Received = 0
msgs_Dropped = 0

prev_val = 0
first = True

totalMessages = 0

# Clear the UART buffer
# test = 0
# while test < 100:
#     if ser.in_waiting > 0:
#         ser.readline().decode('utf-8').strip()
#         test += 1

# Read data from the serial port (example) 
# NOTE TO SELF, simplify type casting by explicitly storing processed_data[2] as an int
while (time.time() - start_time) < duration:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()  # Read and decode data
        processed_data = data.split()
        # print(processed_data)

        if len(processed_data) >= 2:
            totalMessages += 1
            try:
                current_val = float(processed_data[1])
                msgs_Received += 1
                if (current_val != prev_val + 1 and not first):
                    print(prev_val, current_val)
                    # dropped = current_val - prev_val if current_val > prev_val else (256 - prev_val) + current_val
                    msgs_Dropped += current_val - prev_val
                    # print("Messages dropped: ", dropped)
                # if (current_val == prev_val + 1 or first):
                #     msgs_Received += 1
                #     # print(processed_data[2], ": msgs_Received, ", "elapsed:", time.time() - start_time)
                #     first = False
                # else:
                #     msgs_Dropped += 1
                #     # print(processed_data[2], ": msgs_Dropped, ", "elapsed:", time.time() - start_time)
                prev_val = current_val
                first = False
            except ValueError:
                print("ValueError: ", processed_data)
                msgs_Dropped += 1
                # print("msgs_Dropped data format at elapsed:", time.time() - start_time, " data:", processed_data)
bar(msgs_Received, msgs_Dropped)
print("Total messages handled on usb anchor: ", totalMessages)

ser.close()

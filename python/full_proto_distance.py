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

delay1 = sys.argv[1]
delay2 = sys.argv[2]
delay3 = sys.argv[3]

TITLE_TYPE = "3 anchors: 1 responder: " + delay1 + ", "+ delay2 +", "+ delay3 +" ms delay"

# A, B, C = [], [], []
curA, curB, curC = 0, 0, 0
firstA, firstB, firstC = True, True, True
receivedA, receivedB, receivedC = 0, 0, 0
droppedA, droppedB, droppedC = 0, 0, 0
prevA, prevB, prevC = 0, 0, 0
invalidA, invalidB, invalidC = 0, 0, 0

xAxisA, xAxisB, xAxisC = [], [], []
yAxisA, yAxisB, yAxisC = [], [], []

start_time = time.time()
duration = 20  # 20 seconds

# def determineIfDropped(data):
prev_val = 0
totalMessages = 0

def plot_scatter():
    plt.figure()
    plt.scatter(xAxisA, yAxisA, label='A', alpha=0.6)
    plt.scatter(xAxisB, yAxisB, label='B', alpha=0.6)
    plt.scatter(xAxisC, yAxisC, label='C', alpha=0.6)
    plt.xlabel('Time (s)')
    plt.ylabel('Average Distance(meters) per 10 samples')
    plt.title(TITLE_TYPE)
    plt.legend()
    plt.savefig(TITLE_TYPE + '_scatter.png')
    plt.close()

def plot_total_and_dropped(receivedA, receivedB, receivedC, droppedA, droppedB, droppedC):
    labels = ['A', 'B', 'C']
    total = [receivedA, receivedB, receivedC]
    dropped = [droppedA, droppedB, droppedC]
    invalid = [invalidA, invalidB, invalidC]  # Added invalid array

    x = np.arange(len(labels))
    width = 0.25  # Reduced width to accommodate third bar

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width, total, width, label='Received')
    rects2 = ax.bar(x, dropped, width, label='Dropped')
    rects3 = ax.bar(x + width, invalid, width, label='Invalid', color='purple')  # Added invalid bars
    
    # Add separate bars for totals
    total_recv = sum(total)
    total_drop = sum(dropped)
    total_inv = sum(invalid)  # Added total invalid
    ax.bar(len(labels) - width, total_recv, width, label='Total Received', color='green')
    ax.bar(len(labels), total_drop, width, label='Total Dropped', color='red')
    ax.bar(len(labels) + width, total_inv, width, label='Total Invalid', color='purple')  # Added total invalid bar

    ax.set_ylabel('Messages')
    ax.set_title(TITLE_TYPE)
    ax.set_xticks(list(x) + [len(labels)])
    ax.set_xticklabels(labels + ['Total'])
    ax.legend()

    # Add value labels on top of each bar
    def autolabel(rects):
        for rect in rects:
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width()/2., height,
                    f'{int(height)}',
                    ha='center', va='bottom')

    autolabel(rects1)
    autolabel(rects2)
    autolabel(rects3)
    
    # Add labels for total bars
    ax.text(len(labels) - width, total_recv, f'{total_recv}', ha='center', va='bottom')
    ax.text(len(labels), total_drop, f'{total_drop}', ha='center', va='bottom')
    ax.text(len(labels) + width, total_inv, f'{total_inv}', ha='center', va='bottom')

    plt.savefig(TITLE_TYPE + '.png')
    plt.close()

while (time.time() - start_time) < duration:
    if ser.in_waiting > 0:
        totalMessages += 1
        data = ser.readline().decode('utf-8').strip()  # Read and decode data
        processed_data = data.split()
        if totalMessages < 100: # To clear UART BUFFER
            continue
        elif totalMessages > 100 and firstA and firstB and firstC:
            print("Cleared UART buffer")
            start_time = time.time()

        print(processed_data)
        current_val = int(processed_data[2])
        current_dist = float(processed_data[1])
        

        if len(processed_data) == 3:
            case = processed_data[0]
            if float(processed_data[1]) <= 0:
                # print("Invalid distance: ", case, processed_data[1])
                if case == 'A':
                    invalidA += 1
                elif case == 'B':
                    invalidB += 1
                elif case == 'C':
                    invalidC += 1
                continue

            if case == 'A':
                if firstA:
                    firstA = False
                    prevA += current_val
                else:
                    if current_val - prevA > 1:
                        # print("Dropped A: ", prevA, current_val)
                        droppedA += current_val - prevA - 1
                    prevA = current_val
                receivedA += 1
                curA += current_dist
                if receivedA % 10 == 0:
                    xAxisA.append(time.time() - start_time)
                    yAxisA.append(curA / 10.0)
                    curA = 0
            elif case == 'B':
                if firstB:
                    firstB = False
                    prevB = current_val
                else:
                    if current_val - prevB > 1:
                        # print("Dropped B: ", prevB, current_val)
                        droppedB += current_val - prevB - 1
                    prevB = current_val
                receivedB += 1
                curB += current_dist
                if receivedB % 10 == 0:
                    xAxisB.append(time.time() - start_time)
                    yAxisB.append(curB / 10.0)
                    curB = 0
            elif case == 'C':
                if firstC:
                    firstC = False
                    prevC = current_val
                else:
                    if current_val - prevC > 1:
                        # print("Dropped C: ", prevC, current_val)
                        droppedC += current_val - prevC - 1
                    prevC = current_val
                receivedC += 1
                curC += current_dist
                if receivedC % 10 == 0:
                    xAxisC.append(time.time() - start_time)
                    yAxisC.append(curC / 10.0)
                    curC = 0
            else:
                # print("Invalid case: ", case)
                continue
            

plot_total_and_dropped(receivedA, receivedB, receivedC, droppedA, droppedB, droppedC)
plot_scatter()
ser.close()
        
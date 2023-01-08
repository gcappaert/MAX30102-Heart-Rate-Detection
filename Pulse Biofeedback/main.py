#main.py

from max30102 import MAX30102, MAX30105_PULSE_AMP_HIGH, MAX30105_PULSE_AMP_MEDIUM, MAX30105_PULSE_AMP_LOW
from machine import Pin, SoftI2C
import gc
from time import sleep, ticks_us, ticks_diff
from ulab import numpy as np
from math import pi, sin

i2c = SoftI2C(scl=Pin(7), sda=Pin(6), freq=400000)
sensor = MAX30102(i2c=i2c)


sleep(1)
if sensor.i2c_address not in i2c.scan():
    print("Sensor not found.")
elif not (sensor.check_part_id()):
    # Check that the targeted sensor is compatible
    print("I2C device ID not corresponding to MAX30102 or MAX30105.")
else:
    print("Sensor connected and recognized.")

sensor.setup_sensor()

SAMPLE_RATE = 200
FIFO_AVERAGE = 8

sr = SAMPLE_RATE/FIFO_AVERAGE

sensor.set_sample_rate(SAMPLE_RATE)
sensor.set_fifo_average(FIFO_AVERAGE)
sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)
compute_frequency = False


def gather_sensor_data(sensor,bufferlen):
    global data_buffer
    t_start = ticks_us()
    samples_n = 0
    data_buffer = list()
    while len(data_buffer) < bufferlen:
        if sensor.check():
            if sensor.available():
                ir = sensor.pop_ir_from_storage()
                data_buffer.append(ir)
            if compute_frequency:
                if ticks_diff(ticks_us(), t_start) >= 999999:
                    f_HZ = samples_n
                    samples_n = 0
                    print("acquisition frequency = ", f_HZ)
                    t_start = ticks_us()
                else:
                    samples_n += 1

def normalize_data(data):
    data = np.array(data)
    mean = np.mean(data)

    return data - mean

def remove_artifact(data):
    """Handle the artifact caused by the first reading sometimes coming in very high or very low"""
    mean = np.mean(data)
    sd = np.std(data)
    max_deviations = 3
    
    if abs(data[0] - mean) > max_deviations * sd:
        data[0] = data[1]
    
    return data

def remove_baseline(data):
    x = np.arange(len(data))
    polynomial = np.polyfit(x, data,1)
    baseline = x * polynomial[0] + polynomial[1]
    baseline_removed = data-baseline
    return baseline_removed

def detect_peaks(signal):
    """Naive linear search to find the peaks of the algorithm
    
    Returns a tuple of peak indices
    """
    peak_indices = []
    baseline = 0
    peak_value = -999
    peak_index = -999

    for i, value in enumerate(signal):
        if value > baseline:
            if value > peak_value or peak_value == -999:
                peak_index = i
                peak_value = value
        elif value < baseline and peak_value != -999:
            peak_indices.append(peak_index)
            peak_index = -999
            peak_value = -999
    
    if peak_index != -999:
        peak_indices.append(peak_index)

    
    return tuple(peak_indices)

def calc_autocorrelation (data, m):
    mean = np.mean(data)
    ndata = data-mean
    variance = np.sum(ndata**2) / len(data)
    tmp = np.dot(ndata[m:], ndata[:-m])
    c = tmp / len(data) / variance
    return c

def check_peaks(data,peak_indices):
    peak_differences = []
    for i in range(len(peak_indices)-1):
        peak_differences.append(abs(peak_indices[i+1] - peak_indices[i]))
    if len(peak_differences) > 0:
        median_difference = np.median(np.array(peak_differences))
        quality = calc_autocorrelation(data,int(median_difference))
        if quality > 0.25:
            return median_difference
        else:
            return None
    else:
        return None

    
def write_to_csv(filename,data):
    with open(filename, "a+") as file:
        entry = ','.join(str(i) for i in data)
        file.write(entry)
        file.write('\n')


        
for i in range(5):

    gather_sensor_data(sensor,100)
    data = normalize_data(data_buffer)
    data = remove_artifact(data)
    data = remove_baseline(data)
    peak_distance = check_peaks(data,detect_peaks(data))
    
    if peak_distance:
        rate = 1 / (peak_distance/sr) * 60
        print("BPM: {}".format(round(rate,1)))
    else: 
        print("Rate could not estimated with certainty from signal")
    

    write_to_csv("IR_signal.csv",data)




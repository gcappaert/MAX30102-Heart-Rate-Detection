#main.py

from max30102 import MAX30102, MAX30105_PULSE_AMP_HIGH, MAX30105_PULSE_AMP_MEDIUM, MAX30105_PULSE_AMP_LOW
from machine import Pin, SoftI2C
import gc
from time import sleep, ticks_ms
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

sensor.set_sample_rate(SAMPLE_RATE)
sensor.set_fifo_average(FIFO_AVERAGE)

sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)



def gather_sensor_data(sensor,bufferlen):
    global data_buffer
    data_buffer = list()
    while len(data_buffer) < bufferlen:
        if sensor.check():
            if sensor.available():
                ir = sensor.pop_ir_from_storage()
                data_buffer.append(ir)

def normalize_data(data):
    data = np.array(data)
    mean = np.mean(data)

    return data - mean

def suppress_outliers(data):
    mean = np.mean(data)
    sd = np.std(data)
    med = np.median(data)
    distance_from_mean = abs(data - mean)
    max_deviations = 3
    
    low = np.median(data[data < med])
    high = np.median(data[data > med])

    # Deal with the artifact sometimes caused by the first reading coming in way above or way below average as the sensor calibrates its baseline
    if distance_from_mean[0] > max_deviations * sd:
        data[0] = data[1]
        distance_from_mean[0] = distance_from_mean[1]


    # Impute upper quartile for the high outliers and lower quartile for the low outliers

    data[(distance_from_mean > max_deviations * sd) & (data > mean)] = med
    data[(distance_from_mean > max_deviations * sd) & (data < mean)] = low
 
    return data
    
def remove_baseline(data):
    x = np.arange(len(data))
    polynomial = np.polyfit(x, data,deg=1)
    baseline = x * polynomial[0] + polynomial[1]
    baseline_removed = data-baseline
    return baseline_removed

def detect_peaks(signal):
    """Naive linear search to find the peaks of the algorithm
    
    Returns a tuple of peak indices
    """
    peak_indices = []
    baseline = np.median(signal[signal > np.median(signal)])
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

def rate_from_peaks(peak_indices, sample_rate):
    peak_differences = []
    for i in range(len(peak_indices)-1):
        peak_differences.append(abs(peak_indices[i+1] - peak_indices[i]))
    
    rate = np.min(np.array(peak_differences)) / 25 * 60
    return rate
    
def write_to_csv(filename,data):
    with open(filename, "a+") as file:
        entry = ','.join(str(i) for i in data)
        file.write(entry)
        file.write('\n')


        
for i in range(5):

    gather_sensor_data(sensor,100)
    data = normalize_data(data_buffer)
    data = suppress_outliers(data)
    data = remove_baseline(data)
    rate = rate_from_peaks(detect_peaks(data),25)

    print("BPM: {}".format(rate))

    write_to_csv("IR_signal.csv",data)

    # gc.collect()
    # window = 7
    # idx = int((window-1)/2)
    # passes = 4
    
    # filtered_data = moving_average_filter(data, window)
    
    # for i in range(passes-1):
    
    #     filtered_data = moving_average_filter(filtered_data, window)
    
    
    # filtered_data = data[passes*idx:len(data)-passes*idx] - filtered_data
    
    # for i in filtered_data:
    #      print(i)
    #      sleep(0.05)
#     data = remove_baseline_moving_average(data)
#     sr = int(SAMPLE_RATE/FIFO_AVERAGE)
#     bpm = find_fundamental_frequency(data, sr) * 60
#     bpm = round(bpm,0)
#     print("BPM: " + str(bpm))



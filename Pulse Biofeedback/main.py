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



def moving_average_filter(data, m):
    data = np.array(data) / 5
    n = len(data)

    idx = int((m-1)/2)
    filtered_data = np.zeros(len(data)-(m-1))

    first_point = 0

    for i in range(-idx,idx+1):
        first_point += data[idx + i]

    filtered_data[0]

    for i in range(1,len(data)-(m-1)):
        point = filtered_data[i-1] + data[i+(m-1)] - data[i-1]
        filtered_data[i] = round(point,4)

    return filtered_data

def remove_baseline_moving_average(data, m):
    data = np.array(data)
    # replace each data point with the average of neighbor samples in a window of size n
    filtered_data = moving_average_filter(data, m)
    idx = int((m-1)/2)
    return (data[idx:len(data)-idx]-filtered_data)
 
def find_fundamental_frequency(data, sr):
    # To resolve frequencies to a 3 bpm resolution, a waveform frequency resolution of 0.05 Hz is necessary
#     zero_pad = np.zeros(len(data), dtype=np.float)
#     data = np.concatenate((zero_pad,data))
    fft_real, fft_imaginary = np.fft.fft(data)
    time= np.arange(len(data))
    N = len(data)
    T = N / sr
    freq = time / T
    
    freq_magnitude = np.sqrt(fft_real ** 2 + fft_imaginary ** 2)
    
    freq_dist = dict(zip(freq,freq_magnitude))
    freq_dist = { key:value for (key,value) in freq_dist.items() if key > 0.6 and key < 3.0 } 
    
    
    fundamental = max(freq_dist, key=freq_dist.get)
    return fundamental

def calc_autocorrelation (data, m):
    mean = np.mean(data)
    ndata = data-mean
    variance = np.sum(ndata**2) / len(data)
    tmp = np.dot(ndata[m:], ndata[:-m])
    c = tmp / len(data) / variance
    return c
    
def find_max_autocorrelation (data,m=25):
    maximum_lag = 40
    minimum_lag = 6
    aut = calc_autocorrelation(data,m)
    right = calc_autocorrelation(data,m+1)
    if right > aut:
        nlag = m + 2
        while right > aut and nlag < maximum_lag:
            #step to the right until the value starts to get lower
            aut = right
            right = calc_autocorrelation(data,nlag)
            nlag += 1
        return (aut, nlag-2)
    left = calc_autocorrelation(data,m-1)
    if left > aut:
        nlag = m - 2
        while left > aut and nlag > minimum_lag:
            aut = left
            left = calc_autocorrelation(data,nlag)
            nlag -= 1
        return (aut, nlag+2)
    
    return (aut,m)
    
def write_to_csv(filename,data):
    with open(filename, "a+") as file:
        entry = ','.join(str(i) for i in data)
        file.write(entry)
        file.write('\n')


        
for i in range(5):

    gather_sensor_data(sensor,128)
    data = normalize_data(data_buffer)
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



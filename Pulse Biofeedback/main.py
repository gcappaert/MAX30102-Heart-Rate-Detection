#main.py

from max30102 import MAX30102, MAX30105_PULSE_AMP_HIGH, MAX30105_PULSE_AMP_MEDIUM, MAX30105_PULSE_AMP_LOW
from machine import Pin, SoftI2C
import uasyncio
from time import sleep, ticks_ms
from ulab import numpy as np

i2c = SoftI2C(scl=Pin(7), sda=Pin(6), freq=400000)
sensor = MAX30102(i2c=i2c)

data_buffer = list()

sleep(1)
if sensor.i2c_address not in i2c.scan():
    print("Sensor not found.")
elif not (sensor.check_part_id()):
    # Check that the targeted sensor is compatible
    print("I2C device ID not corresponding to MAX30102 or MAX30105.")
else:
    print("Sensor connected and recognized.")

sensor.setup_sensor()

sensor.set_sample_rate(400)
sensor.set_fifo_average(8)

sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_HIGH)

def gather_sensor_data(sensor):
    global data_buffer
    data_buffer = list()
    while len(data_buffer) < 260:
        if sensor.check():
            if sensor.available():
                ir = sensor.pop_ir_from_storage()
                data_buffer.append(ir)

def normalize_data(data):
    data = np.array(data)
    mean = np.mean(data)
#     std = np.std(data)
#     distance_from_mean = abs(data-mean)
#     max_deviations = 2
#     data[distance_from_mean < max_deviations * std]
        
    return data - mean

def remove_baseline_moving_average(data):
    data = np.array(data)
    # replace each data point with the average of neighbor samples in a window of size n 
    n = 4
#     kernel = np.ones((1,n))/n
#     kernel = kernel[0,:]
# 
#     filtered_data = np.convolve(data,kernel)[n-1:n+len(data)-1]
    
    # The length of the new array is n + len(data) - 1, so 131
    # To end up with an array 128 units long, I'll slice the data from 3:130
    
    filtered_data = np.zeros(len(data))

    for i in range(len(data)):
        if i < n-1:
            n+=1
        else:
            n=n
        
        for j in range(n):
            filtered_data[i] += data[i-j]

        filtered_data[i] /= n

    return (data-filtered_data)

def find_fundamental_frequency(data, sr=50):
    zero_pad = np.zeros(256, dtype=np.float)
    data_zeroes = np.concatenate((data,zero_pad))
    fft_real, fft_imaginary = np.fft.fft(data_zeroes)
    time= np.arange(len(data_zeroes))
    N = len(data_zeroes)
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
        


# async def main():
#     #Set up pins
# 
#     i2c = SoftI2C(scl=Pin(7), sda=Pin(6), freq=400000)
#     sensor = MAX30102(i2c=i2c)
# 
#     sleep(1)
#     if sensor.i2c_address not in i2c.scan():
#         print("Sensor not found.")
#         return
#     elif not (sensor.check_part_id()):
#         # Check that the targeted sensor is compatible
#         print("I2C device ID not corresponding to MAX30102 or MAX30105.")
#         return
#     else:
#         print("Sensor connected and recognized.")
# 
#     sensor.setup_sensor()
# 
#     sensor.set_sample_rate(400)
#     sensor.set_fifo_average(8)
# 
#     sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)
        
for i in range(10):

    gather_sensor_data(sensor)
    data = normalize_data(data_buffer[2:258])
#     data = remove_baseline_moving_average(data)
    bpm = find_fundamental_frequency(data) * 60
    bpm = round(bpm,0)
    print("BPM: " + str(bpm))

#     with open ('IR_signal.csv', 'a+') as file:
#         for point in data:
#             print(point)
#             file.write(str(point) + ',')
#         file.write('\n')
        
        
        
#     first_local_maximum = find_max_autocorrelation(data,25)
#     if first_local_maximum[0] > 0.25:
#         bpm = 1500 / first_local_maximum[1]
#         print('Estimate BPM: ' + str(bpm))
#     else:
#         print('Waiting for better signal')

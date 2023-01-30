# main.py
# Tested on RP2040 

# HR/O2 sensor driver
from max30102 import MAX30102, MAX30105_PULSE_AMP_HIGH, MAX30105_PULSE_AMP_MEDIUM, MAX30105_PULSE_AMP_LOW
from machine import Pin, SoftI2C
# Display driver
from ssd1306 import SSD1306_I2C
from ulab import numpy as np
from math import sqrt

# Intialize the sensor
i2c = SoftI2C(scl=Pin(7), sda=Pin(6), freq=400000)
oled = SSD1306_I2C(128,32, i2c)
sensor = MAX30102(i2c=i2c)

# Check that sensor is found at the correct hardware address
# I2C scan is necessary to initialize the sensor on RP2040 
if sensor.i2c_address not in i2c.scan():
    print("Sensor not found.")
elif not (sensor.check_part_id()):
    # Check that the targeted sensor is compatible
    print("I2C device ID not corresponding to MAX30102 or MAX30105.")
else:
    print("Sensor connected and recognized.")

sensor.setup_sensor()

# Choose sampling rate and numer of samples to average. Averaging samples reduces noise
# In theory, many options are available. In practice, I have found that only certain combinations work well

SAMPLE_RATE = 200
FIFO_AVERAGE = 8
sr = SAMPLE_RATE/FIFO_AVERAGE

sensor.set_sample_rate(SAMPLE_RATE)
sensor.set_fifo_average(FIFO_AVERAGE)
sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)
sensor.set_led_mode(2)
sensor.set_pulse_width(411)


# Collect sensor data and store it in a data buffer

def gather_sensor_data(sensor,bufferlen=100):
    global data_buffer
    data_buffer = list()
    while len(data_buffer) < bufferlen:
        if sensor.check():
            if sensor.available():
                # Register addresses for IR and RED data are switched for the cloned MAX 30102 sensor I own
                red = sensor.pop_ir_from_storage()
                ir = sensor.pop_red_from_storage()
                data_buffer.append((ir,red))
                
# Gather one set of dummy data to initialize the sensor

gather_sensor_data(sensor,25)


# Functions to remove DC component, sensor artifact, and center signal around a baseline

def normalize_data(data):
    try:
        return data - np.mean(data)
    except TypeError:
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
    """Linear regression to find data baseline """
    x = np.arange(len(data))
    polynomial = np.polyfit(x, data,1)
    baseline = x * polynomial[0] + polynomial[1]
    baseline_removed = data-baseline
    return baseline_removed

# Functions for detecting peaks and measuring signal quality

# Finds peaks in the signal, correponding to beats

def detect_peaks(signal):
    """Naive linear search to find the peaks in the signal
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
    """A method for measuring signal quality - a good signal should have a high autocorrelation value with a lag equivalent to
    the distance between peaks"""
    mean = np.mean(data)
    ndata = data-mean
    variance = np.sum(ndata**2) / len(data)
    if m == 0:
        tmp = np.dot(ndata,ndata)
    else:
        tmp = np.dot(ndata[m:], ndata[:-m])
    c = tmp / len(data) / variance
    return c

def calc_pearson_correlation(data1,data2):
    """Returns correlation coefficient between two data sets"""
    n = len(data1)
    r = (n * np.dot(data1,data2) - np.sum(data1) * np.sum(data2)) / sqrt((n*np.sum(data1**2)-np.sum(data1)) * (n*np.sum(data2**2)-np.sum(data1)))

    return r


def check_peaks(data,peak_indices):
    '''
    Given peaks of a signal, filters to determine if peak-to-peak distance is a reasonable estimate of the period of the signal.
    Values indicating a heart rate greater than 300 are ruled out, as are those indicating an HR of less than 20
    Values that do not autocorrelate to a certain threshold are ruled out
    Returns the median of the values that meet these requirements
    '''
    peak_differences = []
    threshold = 0.25
    for i in range(len(peak_indices)-1):
        peak_diff = abs(peak_indices[i+1] - peak_indices[i])
        if calc_autocorrelation(data,peak_diff) > threshold and peak_diff > 10:
            peak_differences.append(peak_diff)
    if len(peak_differences) > 0:
        median_difference = np.median(np.array(peak_differences))
        return median_difference
    else:
        return None



    
def write_to_csv(filename,data):
    """Writes data from one channel to a CSV file"""
    with open(filename, "a+") as file:
        entry = ','.join(str(i) for i in data)
        file.write(entry)
        file.write('\n')


        
while True:

    gather_sensor_data(sensor,100)
    ir_data = np.array([x[0] for x in data_buffer])
    red_data = np.array([x[1] for x in data_buffer])
    
    ir_raw_mean, red_raw_mean = np.mean(ir_data), np.mean(red_data)
    ir_data, red_data = normalize_data(ir_data), normalize_data(red_data)
    ir_data, red_data = remove_artifact(ir_data), remove_artifact(red_data)
    ir_data, red_data = remove_baseline(ir_data), remove_baseline(red_data)
    
    peaks = detect_peaks(ir_data)
    peak_distance = check_peaks(ir_data,peaks)

    
    if peak_distance:

        # calculate HR
        rate = 1 / (peak_distance/sr) * 60

        # make sure the Red and IR signals are well correlated
        if calc_pearson_correlation(ir_data,red_data) > 0.8:    
        
            ir_rms = sqrt(np.sum(ir_data[0:int(peak_distance)] ** 2 / len(ir_data)))
            red_rms = sqrt(np.sum(red_data[0:int(peak_distance)] ** 2 / len(red_data)))

            z = (red_rms / red_raw_mean) / (ir_rms / ir_raw_mean)
        
        
        # I tried Maxim's default calibration coeffcients (-45.060 * z * z + -30.354 * z + 94.845). 
        # I have also seen 104-17*z as an estimate, the linear regression corresponding to z values between 0.4 and 3.4
        # The latter seems to better correspond to reality, but is still an imperfect approximation
        # Ideally, this would be calibrated for each application using a reference standard

            saturation = 104 - 17*z
    
            oled.fill(0)
            oled.text("BPM: {}".format(round(rate,1)),0,0)
            oled.text("O2: {}%".format(round(saturation,1)),0,10)
            oled.show()
        else:
            oled.fill(0)
            oled.text("BPM: {}".format(round(rate,1)),0,0)
            oled.text("O2 could not",0,10)
            oled.text("be calculated",0,20)
    else: 
        oled.fill(0)
        oled.text("Awaiting better", 0, 0)
        oled.text("signal.",0,10)
        oled.show()
    




# MAX30102 O2 and HR Detection

This program was written for a RP2040 running Micropython firmware with the fabulous ulab extension (a stripped-down numpy for Micropython). As written at present, it outputs HR and SPO2 data to a display, but the next step is to add an SD card, build a housing and attachment mechanism, and use it for nighttime O2 monitoring. 

It makes use of @n-elia's driver library for the MAX30102 oxygen sensor as well as an SSD1306 driver library. I naively thought it would be trivially easy to obtain good quality HR and SPO2 data from the MAX30102 senor, but this was not the case. I began following the approach used by @aromring in his excellent Instructable tutorial, but deviated somewhat from his approach. His method of checking signal quality using autocorrelation and pearson correlation for the red and IR signals greatly improved accuracy. 

## Pitfalls with MAX30102 sensor and PPG signal processing that may help other users:

* If your math is as remedial as mine, you may be tempted to use a fourier transform to extract the dominant frequency and therefore the heart rate. However, to obtain a reasonably precise estimate (within 3 bpm), you'll need to collect 10+ seconds of data. This is a big downside for a lot of applications, so I'd recommend using a peak detection approach.

* There are a lot of options for sample rate and # of readings to average. Basically, the sensor has a native moving average filter. I found that 200 samples/second with 8 sample average (true sample rate of 25 samples/second) produced a fairly reliable signal without losing too much of the true HR signal. I experimented with other combinations, but found this most effective. Unless you're aiming for a more ambitious use of the PPG signal that requires accuarate measurement of the dicrotic notch (e.g. approximating BP), I'd stick with low-ish sample rates. 

* As @n-elia pointed out in his documentation for his driver library, some of the MAX30102 clones you can buy have the infrared and red registers switched. This was the case for mine.

* As with all optical PPG sensors, ambient light makes a big difference. Housing the sensor in an opaque sleeve will greatly improve signal quality.

## Steps for the future

* ulab presents a lot of interesting possibilities by allowing complex operations to be performed quite quickly. A band pass filter could be implemented fairly easily to clean up the signal. There should also be a way to implement wavelet decomposition, which I think would be the most accurate approach to determine the HR.

* Most of this was written while traveling. When I get home, I'll get the SD card module implemented and set this up for night-time O2 monitoring.

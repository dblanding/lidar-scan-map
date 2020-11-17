"""
ADS1115 datasheet located at:
https://cdn-shop.adafruit.com/datasheets/ads1115.pdf
"""

import time

import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
start = time.time()
last = None
data = []
for n in range(32):
    if not last:
        last = start
    this = time.time()
    delta = this - last
    last = this
    value = adc.read_adc(0, gain=GAIN, data_rate=250)
        # Note you can also pass in an optional data_rate parameter that controls
        # the ADC conversion time (in samples/second). Each chip has a different
        # set of allowed data rate values, see datasheet Table 9 config register
        # DR bit values.
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        # Each value will be a 12 or 16 bit signed integer value depending on the
        # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
    data.append((value, delta))
print(len(data))

for line in data:
    print('{0:>6}'.format(value), 'delta = {:.3}'.format(delta))
    # default speed is 1 reading every 10 mSec
    # Can go faster:
    # value = adc.read_adc(0, gain=GAIN, data_rate=128) # default
    # value = adc.read_adc(0, gain=GAIN, data_rate=250)
    # value = adc.read_adc(0, gain=GAIN, data_rate=475)
    # value = adc.read_adc(0, gain=GAIN, data_rate=860)
    


## TF Mini / TF Mini Plus resources

[Benewake website](http://en.benewake.com/)

### TF Mini

The TFmini is available in two different comm. configurations: I2C & Serial.
The one I have is the Serial type and cannot work in I2C mode.

[Instructables (Complete guide)](https://www.instructables.com/Benewake-LiDAR-TFmini-Complete-Guide/)

[Sparkfun tutorial (I2C)](https://learn.sparkfun.com/tutorials/tfmini---micro-lidar-module-qwiic-hookup-guide)

[Sparkfun tutorial (UART)](https://learn.sparkfun.com/tutorials/tfmini---micro-lidar-module-hookup-guide)

### TF Mini Plus

The TFMini-Plus works optionally in either serial or I2C mode.

[Benewake TFmini_Plus README](https://github.com/TFmini/TFmini-Plus/blob/master/README.md)

[Arduino library for TFMini-Plus using I2C interface](https://github.com/budryerson/TFMini-Plus-I2C)

[Medium article: Interfacing TFmini Plus LiDAR with Raspberry Pi](https://medium.com/@engribrahimqazi/interfacing-tfmini-plus-lidar-with-raspberry-pi-4b-6cd82fcca5f1)

> This last article led me to discover that the TF Mini can send its data directly to the RasPi Python program. No need for an Arduino.  
Instead of using the Arduino to interface with the lidar and then send the data to my python program over the serial bus, I should allow the Python program to interface directly with the TFmini Plus. I took the code in the article and modified it so it doesn't waste time printing, but just collects data as fast as it arrives on the serial port.

```
# -*- coding: utf-8 -*
"""from 'Interfacing TFmini Plus LiDAR with Raspberry Pi 4B' by Ibrahim Qazi
https://medium.com/@engribrahimqazi/interfacing-tfmini-plus-lidar-with-raspberry-pi-4b-6cd82fcca5f1

Needed to enable serial through gpio pins by doing the following:
First: sudo raspi-config, Interfacing Options, then Serial,
select No for the login shell and Yes for the serial port hardware to be enabled.
Afterwards, reboot.
Change serial device from "/dev/ttyAMA0" to "/dev/ttyS0".
"""

import time
import serial

ser = serial.Serial("/dev/ttyS0", 115200)

def read_data():
    """Read data from Benewake TFmini Plus LiDAR and print it"""
    last_time = time.time()
    data = []
    while len(data) < 100:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3]*256
                strength = bytes_serial[4] + bytes_serial[5]*256
                temperature = bytes_serial[6] + bytes_serial[7]*256
                temperature = (temperature/8) - 256
                now = time.time()
                strline = "%s, %s, %s, %s, %s\n" % (counter, distance, strength,
                                                    temperature, now - last_time)
                data.append(strline)
                last_time = now
    with open('tfminiPlus_data', 'w') as f:
        print(len(data))
        f.writelines(data)

if __name__ == "__main__":
    if ser.isOpen() == False:
        ser.open()
    read_data()
```

Examination of the output file, shows that a read occurs as soon as 9 bytes are queued up and waiting on the serial port.  
Also, the elapsed time since the previous read hovers right around 10 mSec, so the device really does operate at 100 Hz, as advertised. 

Bytes waiting, distance, strength, temperature, time since last read

```
9, 28, 5812, 60.0, 0.001964092254638672
9, 28, 5814, 60.0, 0.010060310363769531
9, 28, 5815, 60.0, 0.010145902633666992
9, 28, 5821, 60.0, 0.010141611099243164
9, 28, 5813, 60.0, 0.010251522064208984
9, 28, 5818, 60.0, 0.010085821151733398
9, 28, 5818, 60.0, 0.010092496871948242
9, 28, 5819, 60.0, 0.010134696960449219
9, 28, 5816, 60.0, 0.010140180587768555
9, 28, 5812, 60.0, 0.010195255279541016
9, 28, 5821, 60.0, 0.010090112686157227
...
```
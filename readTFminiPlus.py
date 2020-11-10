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

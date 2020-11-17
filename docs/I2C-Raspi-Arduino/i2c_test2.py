"""
From RASPBERRY PI AND ARDUINO CONNECTED USING I2C
https://oscarliang.com/raspberry-pi-arduino-connected-i2c/
"""
import smbus
import time

bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number

while True:
    var = input("Enter an integer [0-255]: ")
    if not var:
        continue

    writeNumber(int(var))
    print("RPI: Hi Arduino, I sent you ", var)
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print("Arduino: Hey RPI, I received a digit ", number)
    print()

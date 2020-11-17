#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Raspberry Pi to Arduino I2C Communication
# https://www.aranacorp.com/en/communication-between-raspberry-pi-and-arduino-with-i2c/
#i2cdetect -y 1

#library
import sys
import smbus2 as smbus#,smbus2
import time

# Slave Addresses
I2C_SLAVE_ADDRESS = 0x04
I2C_SLAVE2_ADDRESS = 0x05
I2C_SLAVE3_ADDRESS = 0x06

# This function converts a string to an array of bytes.
def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted

def main(args):
    # Create the I2C bus
    # I2Cbus = smbus.SMBus(1)
    with smbus.SMBus(1) as I2Cbus:
        slaveSelect = input("Which Arduino (1-3): ")
        cmd = input("Enter command: ")

        if slaveSelect == "1":
            slaveAddress = I2C_SLAVE_ADDRESS
        elif slaveSelect == "2":
            slaveAddress = I2C_SLAVE2_ADDRESS
        elif slaveSelect == "3":
            slaveAddress = I2C_SLAVE3_ADDRESS
        else:
            # quit if you messed up
            print(slaveSelect== "1")
            print(type(slaveSelect))
            print("no slave selected")
            quit()
        BytesToSend = ConvertStringsToBytes(cmd)
        print("Sent " + str(slaveAddress) + " the " + str(cmd) + " command.")
        print(BytesToSend )
        I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)
        time.sleep(0.5)

        while True:
            try:
                data=I2Cbus.read_i2c_block_data(slaveAddress,0x00,16)
                print("recieve from slave:")
                print(data)
            except:
                print("remote i/o error")
                time.sleep(0.5)
    return 0

if __name__ == '__main__':
     try:
        main(sys.argv)
     except KeyboardInterrupt:
        print("program was stopped manually")
     input()

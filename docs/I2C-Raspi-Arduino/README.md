## I2C communication between RasPi & Arduino

Up until now, the Raspberry Pi has communicated with the Arduino through the **serial bus**, sending three integer values between -255 and +255, separated by commas. The Arduino would respond with a sigle 'A' character. These values are used to specify motor speed & direction. The Arduino (with a motor shield) is very well suited to the task of driving multiple motors, each at an individually specified speed and direction. Much better suited than the Raspberry Pi.

Lately, I have been experimenting with the TFmin LiDAR module, configured to send data to the Arduino, which then relays the data to the RasPi. I used the third of the 3 comma separated values to command the initiation of a scan function, which collected data from the TFmni.

Recently, the TFmini **Plus** LiDAR module came to my attention, advertised to be capable of operating at 100Hz (readings every 10mSec). In my initial experiments, I tried substituting the TFmini **Plus** module for the TFmini module, but wasn't able to achieve faster readings. I then came across a medium article [Interfacing TFmini Plus LiDAR with Raspberry Pi 4B](https://medium.com/@engribrahimqazi/interfacing-tfmini-plus-lidar-with-raspberry-pi-4b-6cd82fcca5f1) which showed a way to hook up the TFmini Plus directly to the RasPi (and did indeed achieve readings every 10mSec), but required the use of the RasPi's only avaliable serial interface.

This leaves open a couple of possibilities:
1. Explore hooking up the TFmini_Plus directly to the RasPi through the I2C bus, and continue to use the Serial bus for the Arduino / RasPi communication.

2. Use the RasPi's only serial bus to communicate with the TFmni Plus and explore using the I2C bus for communicating with the Arduino.

I have explored option 1 but have been unable to find any example code (and get it to work) for a direct I2C bus connection with the RasPi. The [Benewake TFmini website](https://github.com/TFmini) shows it is possible to hook up the TFmini on the I2C bus, but the only code examples I am able to find are for the Arduino. The only way I can find to get it to work directly with the RasPi at (100 Hz) is through the RasPi serial bus.


The second option therefore seems to be the only path forward and is explored below.

### Tutorials on I2C communication between RasPi & Arduino

* > [Raspberry Pi and Arduino connected using I2C](https://oscarliang.com/raspberry-pi-arduino-connected-i2c/) uses python program i2c_test2.py communicating with like named Arduino code.

* > [Raspberry Pi to Arduino I2C Communication](https://www.aranacorp.com/en/communication-between-raspberry-pi-and-arduino-with-i2c/) uses python program raspi_arduino_i2c.py to communicate with like named arduino code.

The first tutorial works well. The Python program prompts the user to enter an integer between 0 and 255, then sends the value to the Arduino, which replies and sends the value back again. (The Arduino code sets its I2C address to 0x04).

Sending a single integer value isn't the same as sending three (comma separated) values (as in my current code)  Thus I looked further to discover the second tutorial which advertises to accept a text string and send it to the Arduino as multiple bytes. The python code in this tutorial uses smbus2, so I needed to install that (needed to use the --user flag).

```pip install --user smbus2```

I wasn't able to get this tutorial working though. One thing I noticed was the Arduino I2C address was set to 0x0b (11), but checking with i2cdetect:

```i2cdetect -y 1```

revealed that 2 addresses were shown: 0x03 and 0x0b. So I modified the code to use address 0x04 instead (like the first tutorial). Now i2cdetect no longer lists 0x03, just 0x04. I thought this might be the reason the second tutorial wasn't working, but it still didn't work.

Intersetingly though, the Arduino sketch of the second tutorial cooperates with the python scripts from both tutorials.

### Path Forward

If I end up using the I2C bus to communicate between the RasPi and the Arduino, I will plan to send two bytes in sequence over the I2C bus to the Arduino. The first byte (an integer between 0 and 7) will specify which motor (1, 2, or 3) and direction (fwd/rev). The second byte (an integer between 16 and 255) will specify the speed. Speeds below 20 are useless for the motors anyway, so any value below 20 can be set to zero.

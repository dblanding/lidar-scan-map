## Mapping LiDAR scan data
This project uses a Benewake TFMini Plus LiDAR module mounted to scan 180 degrees on a vertical axis. A hall effect angle encoder reports the angular position of the scan rotor.

![Zero Turn Car with rotating LiDAR](images/IMG-2104.jpg)

The scan data are collected by a python program running on a Raspberry Pi 3B+.
The python program
* Collects data from the TFminiPlus on the serial port
* Collects readings from the angular encoder on the I2C bus
* Sends commands to the Arduino on the SPI bus
* Finds the best fit lines through the scan points
* Displays a plot of the points and the best fit lines.

The Arduino controls the motors through an Adafruit motor shield

![plot showing data points & best-fit lines](images/scandata.png)

[Things To Do Next](docs/ToDo.md)

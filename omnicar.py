"""This program accesses all the functions of the omni wheel car.

  * Motor drive by command to the Arduino (then through motor shield)
  * Access TFminiPlus data through serial bus
  * Access to rotary angle encoder (through ADC)

The TFminiPlus uses the RasPi's only serial bus for data transfer.
Communication between the Arduino and the Adafruit motor shield (v2.3)
uses the I2C bus.
Attempts to set up the RasPi and Arduino communication over the I2C
bus have been unsuccessful because it interferes with the Arduino
to motor shield communication. To sidestep this problem, commands
are sent from the Raspberry Pi to the Arduino on the SPI bus, with
the Raspberry Pi configured as SPI master, and the Arduino as slave.

Wheel motors:
If the car were to be driven to the right (in the X direction)
as shown in the diagram below:

           ^
           | Y direction
         __|__
        |_____|
        /  |  \
      /    M3   \
  _ /             \ _
 | |               | |
 | |--M1  CAR  M2--| |---> X direction
 |_|               |_|
    \             /
      \    M4   /
        \__|__/
        |_____|

Motor M4 would turn CW and motor M3 CCW at the same speed.
Motors M1 & M2 would be off.

To drive the car sideways (up on the diagram) we would
drive motor M1 CCW and motor M2 CW at the same speed.
Motors M3 & M4 would be off.

To get the car to spin CCW on its own axis, all 4
motors would need to run CW at the same speed.

The cool thing about an omni-wheel car is that all three
of these motions can be superimposed:
    - forward-back motion (X)
    - sideways motion (Y)
    - spin motion (θz).

Each of these 3 independent motions can be controlled
with an independent signal sx, sy, & sz.

Ignoring spinning motion (θz), the signals sx & sy
can be combined vectorally to produce motion in any
oblique direction.

To drive the car at any oblique angle φ:
1. convert rect coords (sx, sy) to polar coords (sr, st)
2. add φ to angle st
3. convert back to rect coords

def r2p(x, y):
    r = math.sqrt(x*x + y*y)
    t = atan(y/x)
    # arctan is tricky... add pi for quadrants 2 & 3
    if (x<0):
        t += math.pi
    return (r, t)

def p2r(r, t):
    x = r * math.cos(t)
    y = r * math.sin(t)
    return (x, y)

To drive the car at 45 degrees, as shown in the diagram below,
it's even simpler: Just combine the X & Y motions

           ^
           | Y dir
         __|__
        |_____|
        /  |  \
      /    M3   \
  _ /             \ _
 | |               | |
 | |--M1  CAR  M2--| |---> X dir
 |_|               |_|
    \             /
      \    M4   / \
        \__|__/    \
        |_____|    _\|
                    fwd dir

go_fwd(spd):
    m4 = spd
    m3 = -spd
    m1 = spd
    m2 = -spd

go_back(spd):
    m4 = -spd
    m3 = spd
    m1 = -spd
    m2 = spd

go_left(spd):
    m4 = spd
    m3 = -spd
    m1 = -spd
    m2 = spd

go_right(spd):
    m4 = -spd
    m3 = spd
    m1 = spd
    m2 = -spd
"""

import Adafruit_ADS1x15
import serial
import spidev
import time

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1  #ADC gain

ser = serial.Serial("/dev/ttyS0", 115200)

spi = spidev.SpiDev()  # Enable SPI
bus = 0  # We only have SPI bus 0 available to us on the Pi
device = 0  # Device is the chip select pin. Set to 0 or 1
spi.open(bus, device)  # Open a connection
spi.max_speed_hz = 500000  # Set SPI speed and mode
spi.mode = 0
spi_wait = .006  # wait time (sec) between successive spi commands


class OmniCar():

    def __init__(self):
        """
        class OmniCar
        Access functions of omni-wheel car.
        Holds distance (cm) last measured by LiDAR module
        """
        self.distance = 0

    def go_oblique1(self, spd):
        """Drive obliquely (quadrant 1) at spd (int between 0-255)."""
        msg1 = [1, spd]
        msg2 = [2+8, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_oblique2(self, spd):
        """Drive obliquely (quadrant 2) at spd (int between 0-255)."""
        msg1 = [4, spd]
        msg2 = [3+8, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_oblique3(self, spd):
        """Drive obliquely (quadrant 3) at spd (int between 0-255)."""
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_oblique4(self, spd):
        """Drive obliquely (quadrant 4) at spd (int between 0-255)."""
        msg1 = [4+8, spd]
        msg2 = [3, spd]
        for msg in (msg1, msg2):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_fwd(self, spd):
        """Drive car forward at speed = spd (int between 0-255)."""
        msg4 = [4, spd]  # High byte, Low byte
        msg3 = [3+8, spd]  # 4th bit in high byte -> reverse dir
        msg1 = [1, spd]
        msg2 = [2+8, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_back(self, spd):
        """Drive car backward at speed = spd (int between 0-255)."""
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_left(self, spd):
        """Drive car left at speed = spd (int between 0-255)."""
        msg4 = [4, spd]
        msg3 = [3+8, spd]
        msg1 = [1+8, spd]
        msg2 = [2, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def go_right(self, spd):
        """Drive car right at speed = spd (int between 0-255)."""
        msg4 = [4+8, spd]
        msg3 = [3, spd]
        msg1 = [1, spd]
        msg2 = [2+8, spd]
        for msg in (msg1, msg2, msg3, msg4):
            _ = spi.xfer(msg)
            time.sleep(spi_wait)

    def spin_ccw(self, spd):
        """Spin car CCW at speed = spd (int between 0-255)."""
        for n in range(1, 5):
            _ = spi.xfer([n, spd])
            time.sleep(spi_wait)

    def spin_cw(self, spd):
        """Spin car CW at speed = spd (int between 0-255)."""
        for n in range(1, 5):
            _ = spi.xfer([n+8, spd])
            time.sleep(spi_wait)

    def run_mtr(self, mtr, spd, rev=False):
        """
        Drive motor = mtr (int 0-7) at speed = spd (int 0-255)
        in reverse direction if rev=True.
        """
        if rev:
            msg = [mtr+8, spd]
        else:
            msg = [mtr, spd]
        _ = spi.xfer(msg)

    def stop_wheels(self):
        """Stop all wheel motors (mtr numbers: 1 thrugh 4)."""
        for n in range(1, 5):
            _ = spi.xfer([n, 0])
            time.sleep(spi_wait)

    def read_dist(self):
        """
        Set self.distance = distance (cm) read from LiDAR module.
        Return number of bytes that were waiting on serial port.
        """
        counter = ser.in_waiting # bytes available on serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                self.distance = bytes_serial[2] + bytes_serial[3]*256
                self.strength = bytes_serial[4] + bytes_serial[5]*256
                temperature = bytes_serial[6] + bytes_serial[7]*256
                self.temperature = (temperature/8) - 256
            ser.flushInput()  # Keep the buffer empty (purge stale data)
        return counter

    def scan_mtr_start(self, spd=None):
        """Turn scan motor on at speed = spd (int between 0-255)."""
        if not spd:
            spd = 200  # default value
        self.run_mtr(7, spd)

    def scan_mtr_stop(self):
        """Turn scan motor off."""
        self.run_mtr(7, 0)

    def scan(self):
        """Return list of tuples of scan data values.
        """
        enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
        # If scan rotor isn't near BDC (back dead cntr), go to BDC
        if enc_val > 3000:
            self.scan_mtr_start()
            while enc_val < 32767:  # continue as values increase to max
                enc_val = adc.read_adc(0, gain=GAIN)
            while enc_val == 32767:  # continue to back dead cntr
                enc_val = adc.read_adc(0, gain=GAIN)
        else:
            self.scan_mtr_start()
            enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
        last_time = time.time()
        data = []
        while enc_val < 32767:  # continue as values increase to max
            enc_val = adc.read_adc(0, gain=GAIN, data_rate=250)
            if 10000 < enc_val < 30000:  # 20000 is 'straight ahead'
                counter = self.read_dist()
                now = time.time()
                delta_t = str(now - last_time)
                last_time = now
                data_item = (enc_val, self.distance, counter, delta_t)
                data.append(data_item)
        self.scan_mtr_stop()
        return data

    def close(self):
        spi.close()
        ser.close()

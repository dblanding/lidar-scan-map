"""
Test script for all motors (4 wheels & lidar rotor).
Consider putting the car up on blocks for this.
"""

import omnicar
import time

car = omnicar.OmniCar()

# Drive car (parameter = motor speed between 0 - 255)
# 'go' values < 90 or 'spin' values < 70 are unreliable due to friction
#

car.go_fwd(200)  # Forward
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.go_back(200)  # Back
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.go_left(90)  # Left
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.go_right(90)  # Right
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.spin_ccw(70)  # CCW 
time.sleep(2)
car.stop_wheels()
time.sleep(.5)

car.spin_cw(70)  # CW
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.close()

"""
Test script for all motors (4 wheels & lidar rotor).
Consider putting the car up on blocks for this.
"""

import omnicar
import time

car = omnicar.OmniCar()

car.go_back(150)  # Backward
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.go_fwd(150)  # Forward
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.go_left(150)  # Left
time.sleep(2)
car.stop_wheels()
time.sleep(1)

car.go_right(150)  # Right
time.sleep(2)
car.stop_wheels()
time.sleep(1)

for n in range(1, 5):
    print(n)
    car.run_mtr(n, 100)
    time.sleep(4)
    car.run_mtr(n+8, 100)
    time.sleep(4)
    car.run_mtr(n, 0)
    time.sleep(.01)

car.scan_mtr_start()
time.sleep(4)
car.scan_mtr_stop()
'''
n = 3
car.run_mtr(n, 150)
time.sleep(4)
car.run_mtr(n+8, 150)
time.sleep(4)
car.run_mtr(n, 0)
'''
time.sleep(1)
car.close()

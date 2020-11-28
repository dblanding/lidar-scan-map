import ztcar
import time

zt = ztcar.OmniCar()
for n in range(1, 5):
    print(n)
    zt.run_mtr(n, 200)
    time.sleep(.04)
time.sleep(2)
for n in range(1, 5):
    zt.run_mtr(n, 0)
    time.sleep(.05)
time.sleep(2)
zt.close()

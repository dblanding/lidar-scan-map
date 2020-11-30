import omnicar
import time

car = omnicar.OmniCar()
for n in range(1, 5):
    print(n)
    car.run_mtr(n, 200)
    time.sleep(.005)
time.sleep(2)
for n in range(1, 5):
    car.run_mtr(n, 0)
    time.sleep(.005)
time.sleep(2)
car.close()

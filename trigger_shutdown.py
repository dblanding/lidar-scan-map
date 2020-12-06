""" Trigger soft shutdown of Pi by grounding shutdown_pin
Configure this script to run at bootup by editing file /etc/rc.local
Add following line to file /etc/rc.local
python /home/pi/trigger_shutdown.py &
"""

import os
import RPi.GPIO as GPIO
import time

shutdown_pin = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def shutdown():
    os.system("sudo shutdown -h now")

while True:
        if not GPIO.input(shutdown_pin):
            shutdown()
        time.sleep(0.5)
try:
    GPIO.wait_for_edge(shutdown_pin, GPIO.FALLING)
    shutdown()

except:
    pass

GPIO.cleanup()

# Car geometery parameters
W2W_DIST = 34  # separation between coaxial wheels (cm)
SONAR_LIDAR_OFFSET = 15  # offset distance between sonar & lidar

# Driving parameters
CARSPEED = 200  # default car speed
SONAR_STOP = 20  # threshold E-stop distance (cm) 
FWD = 94   # forward drive direction (with 4 deg cross-track correction)
LFT = 180  # left drive direction
REV = 270  # reverse drive direction
RGT = 0  # right drive direction

# PID feedback related parameters
KP = 0.6  # steering PID proportional coefficient
KI = 0.1  # steering PID integral coefficient
KD = 1.5  # steering PID derivative coefficient
PIDTRIM = 13  # default value for spin trim
PIDWIN = 6  # number of values to use in rolling average

# Scan related parameters
LEV = 5000  # Low Encoder Value (45-deg behind car's -X direction)
HEV = 30000  # High Encoder Value (car +X direction)
VLEG = 3  # optical path length of vetical leg (cm)
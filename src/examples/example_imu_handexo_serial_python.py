# Description: Example of reading IMU angles from a HandExo device using Python.

import time
from nml_hand_exo import HandExo, SerialComm

# Initialize the HandExo device with the specified serial port and baudrate.
# exo = HandExo(comm='COM6',  baudrate=57600, verbose=False)  

# Serial usage
port = "COM5"
baudrate = 57600

comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=False)


exo.connect()
print("connected")
 
# Wait for the device to be ready.
time.sleep(1)
exo.disable_motor(0)

# Continuously read and print the roll, pitch, and yaw angles from the IMU.
while True:
   # rpy = exo.get_imu_angles()
   rpy = exo.get_imu_heading()
   #  yaw = exo.get_imu_angles()["yaw"]
   print(rpy)
   #  wait = input("bla")
   time.sleep(0.01)
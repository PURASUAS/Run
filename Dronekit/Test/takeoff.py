
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

drone = connect("127.0.0.1:14550", wait_ready=False)



def takeoff(height):
    while drone.is_armable is not True:
        print("Drone is not armable")
        time.sleep(1)


    print("Drone is armable")

    drone.mode = VehicleMode("GUIDED")

    drone.armed = True

    while drone.armed is not True:
        print("Drone is arming...")
        time.sleep(0.5)

    print("Drone has just armed")

    drone.simple_takeoff(height)
    
    while drone.location.global_relative_frame.alt < height * 0.9:
        time.sleep(1)
        print("Drone is rising")

takeoff(10)

for i in range(10):
    print("Waiting :" , str(i))

    time.sleep(1)

# Set mode to guided - this is optional as the goto method will change the mode if needed.
print("LANDING...")
drone.mode = VehicleMode("LAND")

from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to the Vehicle (in this case a simulator running the same computer)
vehicle = connect('127.0.0.1:14550', wait_ready=False)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print( "Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print( "Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print( " Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print( "Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(20)


# Set mode to guided - this is optional as the goto method will change the mode if needed.
vehicle.mode = VehicleMode("GUIDED")

# Set the target location in global-relative frame
a_location = LocationGlobalRelative(-35.36307109 , 149.16517924, 30)
vehicle.simple_goto(a_location)
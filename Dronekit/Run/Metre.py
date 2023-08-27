import math
from dronekit import Command, connect,LocationGlobal, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time


altitude = 10

def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation



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

arm_and_takeoff(altitude)


# Set mode to guided - this is optional as the goto method will change the mode if needed.
vehicle.mode = VehicleMode("GUIDED")

# Set the target location in global-relative frame
y = vehicle.location.global_frame.lon
x = vehicle.location.global_frame.lat
z = vehicle.location.global_frame.alt

newLoc = get_location_metres(LocationGlobal(x,y),10,10)

a_location = LocationGlobalRelative(newLoc.lat , newLoc.lon, altitude)
vehicle.simple_goto(a_location)

print("Going to the direction")

time.sleep(10)

vehicle.mode = VehicleMode("RTL")



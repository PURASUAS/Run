
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import time


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

drone = connect("127.0.0.1:14550", wait_ready=False)


home_x=drone.location.global_frame.lat
home_y=drone.location.global_frame.lon

print("GPS LAT : " ,home_x )
print("GPS LON : " ,home_y )


altitude = 10

a =  get_location_metres(LocationGlobal(home_x,home_y,altitude),10,10)



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
    print("Drone is rising")
    
    while drone.location.global_relative_frame.alt < height * 0.9:
        time.sleep(1)
takeoff(altitude)

# Set mode to guided - this is optional as the goto method will change the mode if needed.
drone.mode = VehicleMode("GUIDED")

# Set the target location in global-relative frame
a_location = LocationGlobalRelative(a.lat, a.lon, altitude)
drone.simple_goto(a_location)


time.sleep(10)

print("Retourning Home ...")

home = LocationGlobalRelative(a.lat, a.lon, altitude)
drone.simple_goto(home)


drone.mode = VehicleMode("LAND")

print("LANDING....")




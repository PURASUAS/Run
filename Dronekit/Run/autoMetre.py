
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
b =    get_location_metres(LocationGlobal(home_x,home_y,altitude),-10,-10)
c =    get_location_metres(LocationGlobal(home_x,home_y,altitude),10,-10)
d =    get_location_metres(LocationGlobal(home_x,home_y,altitude),-10,10)

first_x,first_y =  a.lat,a.lon
second_x,second_y = b.lat, b.lon
third_x,third_y =  c.lat, c.lon

drop_ball_x = d.lat
drop_ball_y = d.lon


def is_arrive(x,y):
    while not( x*0.9<drone.location.global_frame.lat<x*1.1) and not(y*0.9< drone.location.global_frame.lon <y*1.1):
        print("Going to the location...")
        time.sleep(0.5)

    

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


def add_mission():
    global drone_command
    drone_command = drone.commands

    drone_command.clear()
    time.sleep(1)

    # TAKEOFF
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))


    # WAYPOINT
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, first_x,first_y , altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, second_x, second_y, altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, third_x,third_y, altitude))
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home_x ,home_y, altitude))
    

    # VERIFICATION
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home_x,home_y, altitude))

    drone_command.upload()
    print("Commands are loading...")




def drop_ball(x,y):
    global drone_command
    drone_command = drone.commands

    drone_command.clear()
    time.sleep(1)

   
    # WAYPOINT
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x,y, altitude))

    # VERIFICATION
    drone_command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, x,y, altitude))

    drone_command.upload()
    print("Commands are loading...")

   
try:
    takeoff(altitude)

    add_mission()

    drone_command.next = 0

    drone.mode = VehicleMode("AUTO")# Necessary for load commands


    way_point =0


    while True:
        next_waypoint = drone_command.next

        if next_waypoint== way_point:
            way_point+=1
            print("Current Command : ",next_waypoint)

        if next_waypoint ==5 :
            break

    drop_ball( drop_ball_x , drop_ball_y)

    drone_command.next = 0
    print(drone.mode)
    while True:
        next_waypoint = drone_command.next

        if next_waypoint ==0:
            print("Flying through the person...")
            time.sleep(2)
        elif next_waypoint ==1:
            print("Returning home ")
            break
    

finally:

    drone.mode = VehicleMode("RTL")
    print("Drone has returned the launch ")
from dronekit import connect

# Connect to the Vehicle (in this case a simulator running the same computer)
vehicle = connect('127.0.0.1:14550', wait_ready=False)


print(vehicle.location._lat)
print(vehicle.location._lon)

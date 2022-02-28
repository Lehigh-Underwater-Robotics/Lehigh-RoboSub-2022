from sys import _current_frames
from pymavlink import mavutil
import time
import argparse
import math


# Create the connection

# Wait a heartbeat before sending commands
#master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')


def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6),  # Unix time in microseconds
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)


def manualControl(x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # x
        y,  # y
        z,  # z
        r,  # r
        0)  # buttons


def getDepth():
    depth = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                depth = data[5].split(",")[0]
            except:
                print('')
            print("Current Depth: ", depth)

        if not depth == 0:
            break
    return float(depth)


def get_velocity():
    velocity = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                speed = data[2].split(",")[0]
            except:
                print('')

            velocity = float(speed)
        if not velocity == 0:
            break

    return velocity


def get_heading():
    heading = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                heading = data[3].split(",")[0]
            except:
                print('')

            heading = float(heading)
            break

    return heading


def goDepth(depth):
    mode = 'STABILIZE'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    
    current_depth = abs(getDepth())

    if current_depth > depth:
        for i in range(1000000):
            manualControl(0,0,700, 0)
            current_depth = abs(getDepth())

            if current_depth < depth *0.95:
                break
        
        print("REACHED DESIRED Depth: ", getDepth())

        mode = 'ALT_HOLD'
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    else:
        for i in range(1000000):
            manualControl(0,0,300, 0)
            current_depth = abs(getDepth())

            if current_depth > depth *0.95:
                break
        print("REACHED DESIRED Depth: ", getDepth())

        mode = 'ALT_HOLD'
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
    


    
    


def travel_in_x(xThrottle, distanceTravel):
    mode = 'STABILIZE'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")
    start = time.time()
    velocity_array = []
    distance = 0
    for i in range(10000000):
        manualControl(xThrottle, 0, 500, 0)
        elapsed = time.time() - start

        velocity_array.append(get_velocity())
        average_velocity = sum(velocity_array) / len(velocity_array)

        distance = elapsed * average_velocity
        print("RECORDED DISTANCE: ", distance)
        if distance > 0.95*distanceTravel:
            break

    print("REACHED DESIRED DISTANCE: ", distance)

    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def rotateClockwise(degrees):
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    start_heading = get_heading()
    for i in range(10000000):
        current_heading = get_heading()
        rotation = abs(start_heading-current_heading)
        manualControl(0, 0, 500, 250)
        if rotation > 0.96 * degrees:
            break

    print("ROTATED: ", rotation)
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def rotateCounterClockwise(degrees):
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    start_heading = get_heading()
    for i in range(10000000):
        current_heading = get_heading()
        print("Current heading: " , current_heading)
        rotation = abs(start_heading-current_heading)
        manualControl(0, 0, 500, -250)
        if rotation > 0.96 * degrees:
            break

    print("ROTATED: ", rotation)
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def maintainHeading(heading):
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    
    start_heading = get_heading()
    angle = start_heading - heading

    if angle < 0: 
        rotateClockwise(abs(angle))
    else: 
        rotateCounterClockwise(angle)
    
        

        

    
    




wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
boot_time = time.time()
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


master.arducopter_arm()
time.sleep(1)
print("<<<<<<ARMED>>>>>>")


maintainHeading(185)
travel_in_x(1000, 2.5)
rotateCounterClockwise(82)
travel_in_x(1000, 2.5)
rotateCounterClockwise(82)
travel_in_x(1000, 2.5)
rotateCounterClockwise(82)
travel_in_x(1000, 2.5)





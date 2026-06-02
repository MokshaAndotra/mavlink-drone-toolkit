#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  6 17:55:28 2025

@author: moksha
"""

from pymavlink import mavutil
import time

# Mode map for the vehicle (extend this list with more modes as needed)
MODE_MAP = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    8: "POSITION",
    9: "LAND",
    10: "OF_LOITER",
    11: "DRIFT",
    12: "SPORT",
    13: "FOLLOW",
    14: "ZIGZAG",
    15: "SYSTEMID",
    16: "TAKEOFF",  # Added this, just as an example
    17: "RETURN_TO_LAUNCH",  # Added as another example
    # Add other modes as necessary based on your specific use case
}


def connect_vehicle(connection_string):
    """
    Connect to the vehicle.
    """
    print("Connecting to vehicle...")
    the_connection = mavutil.mavlink_connection(connection_string)
    the_connection.wait_heartbeat()
    print("Heartbeat received!")
    return the_connection

def parse_waypoint_file(file_path):
    """
    Parse a waypoint file in Mission Planner format.
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Invalid waypoint file format")
    
    waypoints = []
    for line in lines[1:]:  # Skip the header
        parts = line.strip().split('\t')
        seq, current, frame, command = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
        param1, param2, param3, param4 = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
        lat, lon, alt, autocontinue = float(parts[8]), float(parts[9]), float(parts[10]), int(parts[11])
        
        waypoint = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=1,
            target_component=1,
            seq=seq,
            frame=frame,
            command=command,
            current=current,
            autocontinue=autocontinue,
            param1=param1, param2=param2, param3=param3, param4=param4,
            x=lat, y=lon, z=alt
        )
        waypoints.append(waypoint)
    
    return waypoints

def upload_mission(the_connection, waypoints):
    """
    Upload waypoints to the vehicle using MISSION_ITEM_INT.
    """
    print("Uploading mission...")

    # Send the number of waypoints
    the_connection.mav.mission_count_send(
        the_connection.target_system,
        the_connection.target_component,
        len(waypoints)
    )

    # Send each waypoint using MISSION_ITEM_INT
    for waypoint in waypoints:
        the_connection.mav.mission_request_int_send(
            the_connection.target_system,
            the_connection.target_component,
            waypoint.seq
        )
        print(f"Requesting waypoint {waypoint.seq}")
        
        # Send each waypoint data after receiving MISSION_REQUEST_INT
        the_connection.mav.mission_item_int_send(
            the_connection.target_system,
            the_connection.target_component,
            waypoint.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Use the appropriate frame
            waypoint.command,
            waypoint.current,
            waypoint.autocontinue,
            waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4,
            int(waypoint.x * 1e7),  # Latitude scaled to int32
            int(waypoint.y * 1e7),  # Longitude scaled to int32
            waypoint.z  # Altitude
        )

    print("Mission upload complete!")




def start_mission(the_connection, waypoints):
    """
    Arm the vehicle and start the mission.
    """
    # Arm the drone
    the_connection.set_mode('GUIDED')
    print("Arming the vehicle...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm (1 to arm, 0 to disarm)
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

    # Wait for the vehicle to arm
    while True:
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Vehicle armed!")
            break
        print("Waiting for arming...")
        time.sleep(1)

    # Send a TAKEOFF command explicitly at the first waypoint if needed
    print("Sending takeoff command...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0,  # Minimum pitch
        0,  # Unused
        0,  # Unused
        0,  # Unused
        0,  # Unused
        10,  # Altitude (in meters)
        0   # Unused
    )

    # Set mode to AUTO after sending takeoff command
    print("Setting mode to AUTO...")
    the_connection.set_mode('AUTO')

    # Wait until the mode is set before starting the mission
    time.sleep(2)

    # Start the mission
    print("Starting mission...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,  # Confirmation
        0,  # First mission item index
        0, 0, 0, 0, 0, 0  # Unused parameters
    )


def monitor_and_change_mode(the_connection, mode_changes):
    print("Monitoring mission for mode changes...")
    while True:
        msg = the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        if msg:
            seq = msg.seq
            print(f"Reached waypoint {seq}")

            if seq in mode_changes:
                new_mode = mode_changes[seq]
                print(f"Changing mode to {new_mode} at waypoint {seq}")
                the_connection.set_mode(new_mode)
                time.sleep(5)

                if new_mode != 'AUTO':
                    print("Switching back to AUTO to resume mission")
                    the_connection.set_mode('AUTO')
                    time.sleep(2)

            if seq >= max(mode_changes.keys()):
                print("Reached last monitored waypoint.")
                break






def wait_for_disarm(the_connection):
    print("Waiting for mission to complete (DISARM)...")
    while True:
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Mission complete! Vehicle disarmed.")
                break
        time.sleep(1)




def change_mode_at_waypoint(the_connection, mode, seq):
    """
    Change mode at a specific waypoint (using seq as reference).
    """
    print(f"Waypoint {seq} requests mode {mode}")
    
    if mode == 16:  # Mode '16' corresponds to Takeoff (you can customize the mapping)
        print(f"Sending TAKEOFF command at waypoint {seq}")
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation (set to 0)
            0,  # Param1: Minimum pitch (unused, set to 0)
            0,  # Param2: Empty (unused, set to 0)
            0,  # Param3: Empty (unused, set to 0)
            0,  # Param4: Empty (unused, set to 0)
            0,  # Param5: Empty (unused, set to 0)
            10,  # Param6: Takeoff altitude (e.g., 10 meters)
            0   # Param7: Empty (unused, set to 0)
        )
        time.sleep(2)  # Wait for takeoff to initiate
    else:
        # For other modes, change the mode as usual
        mode_string = MODE_MAP.get(mode, None)
        if mode_string:
            print(f"Changing mode to {mode_string} at waypoint {seq}")
            the_connection.set_mode(mode_string)
            time.sleep(2)
        else:
            print(f"Unknown mode number {mode} at waypoint {seq}. Mode change skipped.")


def main():
    connection_string = 'udp:127.0.0.1:14550'
    waypoint_file_path = "/home/moksha/Downloads/ai_lap_modified.waypoints"  # Updated waypoint file path

    # Connect to the vehicle
    the_connection = connect_vehicle(connection_string)

    # Parse the waypoint file
    waypoints = parse_waypoint_file(waypoint_file_path)

    # Upload the mission
    upload_mission(the_connection, waypoints)

    # Start the mission
    start_mission(the_connection, waypoints)
    
    # Define mode changes at specific waypoints
    mode_changes = {
    5: 'LOITER',
    10: 'GUIDED',
    17: 'RTL'  
    }

  
    
    # Monitor progress and apply mode changes
    monitor_and_change_mode(the_connection, mode_changes)
    
    # Optionally, wait until disarmed
    wait_for_disarm(the_connection)

    
    
    
    

    
    
    

if __name__ == "__main__":
    main()

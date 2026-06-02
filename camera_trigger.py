#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 18:29:23 2025

@author: moksha
"""

from pymavlink import mavutil
import time
import cv2
import threading

camera_active = False

def connect_vehicle(connection_string):
    """
    Connect to the vehicle.
    """
    print("Connecting to vehicle...")
    the_connection = mavutil.mavlink_connection(connection_string)
    the_connection.wait_heartbeat()
    print("Heartbeat received!")
    return the_connection

def start_camera():
    """Non-blocking camera operation in a thread."""
    global camera_active
    print("Starting the camera...")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open the camera.")
        return

    while camera_active:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        cv2.imshow('Camera Feed', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Camera stopped.")

def parse_waypoint_file(file_path):
    """
    Parse a waypoint file in Mission Planner format.
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Invalid waypoint file format")
    
    waypoints = []
    for line in lines[1:]:
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
    Upload waypoints to the vehicle.
    """
    print("Uploading mission...")
    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, len(waypoints))
    for waypoint in waypoints:
        the_connection.mav.mission_request_int_send(the_connection.target_system, the_connection.target_component, waypoint.seq)
        print(f"Requesting waypoint {waypoint.seq}")
        the_connection.mav.mission_item_int_send(
            the_connection.target_system,
            the_connection.target_component,
            waypoint.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            waypoint.command,
            waypoint.current,
            waypoint.autocontinue,
            waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4,
            int(waypoint.x * 1e7),
            int(waypoint.y * 1e7),
            waypoint.z
        )
    print("Mission upload complete!")

def start_mission(the_connection):
    """
    Arm the vehicle and start the mission.
    """
    the_connection.set_mode('GUIDED')
    print("Arming the vehicle...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Vehicle armed!")
            break
        print("Waiting for arming...")
        time.sleep(1)
    
    print("Setting mode to AUTO...")
    the_connection.set_mode('AUTO')
    time.sleep(2)
    print("Starting mission...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def monitor_waypoints(the_connection):
    """
    Monitor waypoints and trigger camera actions.
    """
    global camera_active
    try:
        while True:
            msg = the_connection.recv_match(blocking=True)
            if msg:
                print(f"[{msg.get_type()}] {msg}")
            msg_wp = the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if msg_wp:
                reached_wp = msg_wp.seq
                print(f"Reached Waypoint Index: {reached_wp}")
                if reached_wp == 16 and not camera_active:
                    camera_active = True
                    threading.Thread(target=start_camera).start()
                if reached_wp == 21 and camera_active:
                    print("Stopping camera at waypoint 21...")
                    camera_active = False
                    break
    except KeyboardInterrupt:
        print("Monitoring stopped.")
        camera_active = False

def main():
    connection_string = "udp:127.0.0.1:14550"
    waypoint_file_path = "/home/moksha/Downloads/ai_lap_modified.waypoints"
    
    the_connection = connect_vehicle(connection_string)
    waypoints = parse_waypoint_file(waypoint_file_path)
    upload_mission(the_connection, waypoints)
    start_mission(the_connection)
    monitor_waypoints(the_connection)

if __name__ == "__main__":
    main()
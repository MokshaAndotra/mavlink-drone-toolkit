#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 19:44:29 2025

@author: moksha
"""

from pymavlink import mavutil
import threading
import cv2
import time

camera_running = False

def connect_to_drone(conn_str):
    print("Connecting to drone...")
    conn = mavutil.mavlink_connection(conn_str)
    conn.wait_heartbeat()
    print("Heartbeat received.")
    return conn

def camera_stream(device_index=0, resolution=(640, 480)):
    global camera_running
    cap = cv2.VideoCapture(device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    if not cap.isOpened():
        print("Failed to open camera.")
        return

    print("Camera streaming started.")
    while camera_running:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed.")
            break
        cv2.imshow('Live Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Camera streaming stopped.")

def load_waypoints(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Invalid waypoint file format.")

    waypoints = []
    for line in lines[1:]:
        parts = line.strip().split('\t')
        seq, current, frame, command = map(int, parts[:4])
        param1, param2, param3, param4 = map(float, parts[4:8])
        lat, lon, alt = map(float, parts[8:11])
        autocontinue = int(parts[11])

        waypoint = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=1,
            target_component=1,
            seq=seq,
            frame=frame,
            command=command,
            current=current,
            autocontinue=autocontinue,
            param1=param1, param2=param2,
            param3=param3, param4=param4,
            x=lat, y=lon, z=alt
        )
        waypoints.append(waypoint)

    return waypoints

def send_mission(conn, waypoints):
    print("Uploading mission...")
    conn.mav.mission_count_send(conn.target_system, conn.target_component, len(waypoints))

    for wp in waypoints:
        while True:
            req = conn.recv_match(type='MISSION_REQUEST', blocking=True, timeout=3)
            if req and req.seq == wp.seq:
                conn.mav.mission_item_send(
                    conn.target_system,
                    conn.target_component,
                    wp.seq,
                    wp.frame,
                    wp.command,
                    wp.current,
                    wp.autocontinue,
                    wp.param1, wp.param2, wp.param3, wp.param4,
                    wp.x, wp.y, wp.z
                )
                print(f"Sent waypoint {wp.seq}")
                break

    print("Mission upload complete.")

def arm_drone(conn, timeout=10):
    print("Arming drone...")
    conn.set_mode('GUIDED')
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone armed.")
            return True
        print("Waiting for arming...")
        time.sleep(1)

    print("Failed to arm drone.")
    return False

def initiate_mission(conn):
    print("Switching to AUTO mode and starting mission...")
    conn.set_mode('AUTO')
    time.sleep(2)
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Mission started.")

def wait_for_landing(conn):
    print("Waiting for drone to land and disarm...")
    while True:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Drone disarmed. Mission complete.")
            break
        time.sleep(1)

def monitor_mission(conn, trigger_on=5, stop_on=10, final_wp=17):
    global camera_running
    print("Monitoring mission progress...")
    try:
        while True:
            msg = conn.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if msg:
                wp_index = msg.seq
                print(f"Reached waypoint: {wp_index}")

                if wp_index == trigger_on and not camera_running:
                    print("Starting camera...")
                    camera_running = True
                    threading.Thread(target=camera_stream).start()

                if wp_index == stop_on and camera_running:
                    print("Stopping camera...")
                    camera_running = False

                if wp_index == final_wp:
                    print("Final waypoint reached. Ending monitoring.")
                    break
    except KeyboardInterrupt:
        print("Monitoring interrupted by user.")
        camera_running = False

def main():
    conn_str = "udp:127.0.0.1:14550"
    wp_file = "/home/moksha/Downloads/ai_lap_modified.waypoints"

    conn = connect_to_drone(conn_str)
    waypoints = load_waypoints(wp_file)
    send_mission(conn, waypoints)

    final_waypoint_seq = waypoints[-1].seq  # Get last waypoint number

    if arm_drone(conn):
        initiate_mission(conn)
        monitor_mission(conn, trigger_on=5, stop_on=10, final_wp=final_waypoint_seq)
        wait_for_landing(conn)
    else:
        print("Aborting mission due to arming failure.")

if __name__ == "__main__":
    main()
# MAVLink Drone Toolkit

A Python-based UAV automation and mission planning toolkit built with MAVLink and ArduPilot SITL for mission control, waypoint management, dynamic mode switching, autonomous mission execution, and drone simulation workflows.

## SITL Setup

Launch ArduPilot SITL:

```bash
sim_vehicle.py -v ArduCopter --custom-location=13.0309905,77.565497,10,0 --map --console
```

The scripts communicate with ArduPilot SITL through MAVLink UDP connections for mission execution, waypoint management, and autonomous flight testing.

## Included Tools

### Mission Mode Controller

* Uploads waypoint missions to ArduPilot
* Arms the vehicle and starts autonomous flight
* Supports waypoint-triggered mode changes

### Mission Camera Controller

* Executes autonomous missions
* Starts and stops camera streaming at specified waypoints
* Uses OpenCV for real-time video capture

### KML/KMZ to Waypoint Generator

* Converts KML/KMZ boundary files into Mission Planner waypoint missions
* Generates optimized lawnmower coverage paths
* Supports configurable altitude, spacing, and speed


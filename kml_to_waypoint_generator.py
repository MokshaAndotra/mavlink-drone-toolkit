# -*- coding: utf-8 -*-
"""
Created on Fri Sep 12 12:58:18 2025

@author: hp
"""
import os
import zipfile
import xml.etree.ElementTree as ET
from shapely.geometry import Point, Polygon, LineString, MultiLineString
from shapely.affinity import rotate
from datetime import datetime
import geopandas as gpd
from pyproj import CRS
import math

# Configurable output
OUTPUT_FOLDER = r"C:\Users\hp\OneDrive\Documents\read kml"
OUTPUT_WAYPOINTS_TEMPLATE = "flight_path_{alt}m_{timestamp}.waypoints"


def extract_kml_from_kmz(kmz_file):
    with zipfile.ZipFile(kmz_file, 'r') as kmz:
        for name in kmz.namelist():
            if name.endswith('.kml'):
                return kmz.read(name).decode('utf-8')
    return None


def read_kml_points_from_string(kml_string):
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    root = ET.fromstring(kml_string)
    points = []

    for coord in root.findall(".//kml:coordinates", ns):
        if coord is not None and coord.text:
            coords = coord.text.strip().split()
            for c in coords:
                try:
                    lon, lat, *_ = map(float, c.split(','))
                    points.append(Point(lon, lat))
                except ValueError:
                    continue
    return points


def read_kml_points_from_file(kml_file):
    with open(kml_file, 'r', encoding='utf-8') as f:
        kml_string = f.read()
    return read_kml_points_from_string(kml_string)


def get_utm_crs(lat, lon):
    utm_zone = int((lon + 180) / 6) + 1
    hemisphere = 'north' if lat >= 0 else 'south'
    return CRS.from_dict({
        'proj': 'utm',
        'zone': utm_zone,
        'datum': 'WGS84',
        'south': hemisphere == 'south'
    })


def project_to_utm(geom):
    centroid = geom.centroid
    utm_crs = get_utm_crs(centroid.y, centroid.x)
    gdf = gpd.GeoDataFrame(geometry=[geom], crs='EPSG:4326')
    gdf_utm = gdf.to_crs(utm_crs)
    return gdf_utm.geometry[0], utm_crs


def generate_lawnmower_path(polygon_utm, spacing):
    def get_longest_edge_angle(polygon):
        coords = list(polygon.exterior.coords)
        max_len = 0
        best_angle = 0
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length > max_len:
                max_len = length
                angle_rad = math.atan2(dy, dx)
                best_angle = math.degrees(angle_rad)
        return best_angle

    angle = get_longest_edge_angle(polygon_utm)
    print(f"Using angle based on longest side: {angle:.2f}°")

    rotation_origin = polygon_utm.centroid

    # Rotate polygon so longest side is horizontal
    polygon_rot = rotate(polygon_utm, -angle, origin=rotation_origin)
    minx, miny, maxx, maxy = polygon_rot.bounds
    height = maxy - miny

    # Determine number of sweep lines
    num_lines = math.ceil(height / spacing)
    actual_spacing = height / max(1, num_lines)

    overshoot = spacing * 2
    lines = []

    for i in range(num_lines + 1):
        y = miny + i * actual_spacing
        sweep_line = LineString([(minx - overshoot, y), (maxx + overshoot, y)])
        clipped = sweep_line.intersection(polygon_rot)

        if clipped.is_empty:
            continue

        if isinstance(clipped, LineString):
            coords = list(clipped.coords)
        elif isinstance(clipped, MultiLineString):
            coords = list(max(clipped.geoms, key=lambda g: g.length).coords)
        else:
            continue

        if i % 2 == 1:
            coords.reverse()

        lines.extend(coords)

    if not lines:
        print("No valid lines found.")
        return LineString([])

    path_rot = LineString(lines)
    path = rotate(path_rot, angle, origin=rotation_origin)
    return path


def save_waypoints(path, output_file, altitude, speed):
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        f.write("QGC WPL 110\n")

        lat0, lon0 = path.coords[0][1], path.coords[0][0]
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{lat0}\t{lon0}\t{altitude}\t1\n")

        # Takeoff
        f.write(f"1\t0\t3\t22\t0\t0\t0\t0\t{lat0}\t{lon0}\t{altitude}\t1\n")

        # Change speed
        f.write(f"2\t0\t3\t178\t1\t{speed}\t-1\t0\t0\t0\t0\t1\n")

        # Navigation waypoints
        for i, (x, y) in enumerate(path.coords):
            f.write(f"{i + 3}\t0\t3\t16\t0\t0\t0\t0\t{y}\t{x}\t{altitude}\t1\n")

        # Return to launch
        rtl_index = len(path.coords) + 3
        f.write(f"{rtl_index}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n")

    print(f"Waypoints saved to: {output_file}")


def main(input_file, altitude=200, spacing=15, speed=5):
    print(f"Loading: {input_file}")
    if input_file.lower().endswith('.kmz'):
        kml_str = extract_kml_from_kmz(input_file)
        if not kml_str:
            print("Error reading KMZ")
            return
        points = read_kml_points_from_string(kml_str)
    elif input_file.lower().endswith('.kml'):
        points = read_kml_points_from_file(input_file)
    else:
        print("Unsupported file type")
        return

    if len(points) < 3:
        print("Not enough points to create polygon.")
        return

    coords = [(pt.x, pt.y) for pt in points]
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    polygon = Polygon(coords)

    if not polygon.is_valid:
        polygon = polygon.buffer(0)

    polygon_utm, utm_crs = project_to_utm(polygon)
    path = generate_lawnmower_path(polygon_utm, spacing)

    if path.is_empty:
        print("Path generation failed.")
        return

    path_wgs84 = gpd.GeoSeries([path], crs=utm_crs).to_crs("EPSG:4326").iloc[0]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(
        OUTPUT_FOLDER,
        OUTPUT_WAYPOINTS_TEMPLATE.format(alt=altitude, timestamp=timestamp)
    )

    save_waypoints(path_wgs84, output_file, altitude, speed)


if __name__ == "__main__":
    input_file = 'home/moksha/Downloads/kml.py'
    main(input_file, altitude=100, spacing=30, speed=5)

# Dronekit-Zigzag-Coverage

Generates and uploads an **optimized zigzag (lawnmower) flight path** for drone area coverage missions inside a Pixhawk geofence polygon.

- Reads the geofence automatically from existing mission waypoints on the Pixhawk
- Tests 180 different heading angles (0–179°) and selects the one with the shortest total path length
- Supports configurable altitude, line spacing, and delay at each waypoint
- Uses UTM projection for local accurate planning
- Shows a matplotlib plot of the resulting path

## Important – Before You Start

1. Upload a closed polygon geofence to your Pixhawk first  
   (use Mission Planner, QGroundControl, etc.)
2. Connect your Pixhawk via USB or telemetry radio
3. **Edit the script** — change the connection line to match your setup:
   ```python
   vehicle = connect('COM21', baud=57600, wait_ready=True)


# Pixhawk Zigzag Area Coverage Path Planner

Generates an efficient zigzag (lawnmower) flight path inside a geofence polygon uploaded to Pixhawk.  
Tries 180 different angles and picks the one with shortest total distance.

## Features
- Reads geofence from existing mission waypoints on Pixhawk
- Optimizes heading angle (0–179°) for minimal path length
- Adds configurable delay at each waypoint (good for camera/photos/sampling)
- Uses UTM projection internally → accurate for small areas
- Plots the result with matplotlib

## Important – before running
1. Upload a closed polygon geofence to Pixhawk first (via Mission Planner / QGC)
2. Connect Pixhawk via USB/telemetry
3. Change `'COM21'` and `baud=57600` to match your setup

## Full script (copy-paste ready)

```python
from collections import abc
import collections
collections.MutableMapping = abc.MutableMapping

from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
from shapely.geometry import Polygon, LineString
from shapely.affinity import rotate
from pyproj import Transformer
import matplotlib.pyplot as plt
import math
import time

print(" Connecting to Pixhawk...")
vehicle = connect('COM21', baud=57600, wait_ready=True)
print(" Connected. Vehicle mode:", vehicle.mode.name)

vehicle.commands.download()
vehicle.commands.wait_ready()

mission_items = list(vehicle.commands)
geofence_coords = [
    (cmd.x, cmd.y) for cmd in mission_items
    if cmd.frame in [3, 0] and (cmd.x, cmd.y) != (0.0, 0.0)
]

altitude = 15       # meters
spacing = 10        # meters
delay_seconds = 3   # Delay at each waypoint

if not geofence_coords:
    print(" No valid geofence coordinates received.")
    vehicle.close()
    exit()

# Coordinate transformation
utm_zone = math.floor((geofence_coords[0][1] + 180) / 6) + 1
utm_code = f"epsg:326{utm_zone}"

transformer_fwd = Transformer.from_crs("epsg:4326", utm_code, always_xy=True)
transformer_inv = Transformer.from_crs(utm_code, "epsg:4326", always_xy=True)

utm_coords = [transformer_fwd.transform(lon, lat) for lat, lon in geofence_coords]
if utm_coords[0] != utm_coords[-1]:
    utm_coords.append(utm_coords[0])

polygon = Polygon(utm_coords)

# ── Zigzag generator ────────────────────────────────────────
def generate_zigzag(polygon, angle_deg, spacing):
    rotated = rotate(polygon, -angle_deg, origin=(0, 0), use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds
    lines = []
    y = miny - spacing
    while y <= maxy + spacing:
        line = LineString([(minx - spacing, y), (maxx + spacing, y)])
        inter = rotated.intersection(line)
        if inter.is_empty:
            y += spacing
            continue
        segments = []
        if inter.geom_type == "MultiLineString":
            for seg in inter:
                pts = list(seg.coords)
                segments.append(pts)
        elif inter.geom_type == "LineString":
            pts = list(inter.coords)
            segments.append(pts)
        if segments:
            longest = max(segments, key=lambda s: LineString(s).length)
            lines.append(longest)
        y += spacing
    final_path = [pt for segment in lines for pt in segment]
    rotated_back = rotate(LineString(final_path), angle_deg, origin=(0, 0), use_radians=False)
    return list(rotated_back.coords)

# ── Find best angle ─────────────────────────────────────────
best_path = None
min_distance = float('inf')
best_angle = None

for angle in range(0, 180):
    path = generate_zigzag(polygon, angle, spacing)
    if len(path) < 2:
        continue
    total_dist = sum(math.dist(path[i-1], path[i]) for i in range(1, len(path)))
    if total_dist < min_distance:
        min_distance = total_dist
        best_path = path
        best_angle = angle

# ── Convert back to lat/lon/alt ─────────────────────────────
best_waypoints = [
    (round(transformer_inv.transform(x, y)[1], 9),
     round(transformer_inv.transform(x, y)[0], 9),
     altitude)
    for x, y in best_path
]

print(f"\n Best angle: {best_angle} degrees")
print(f"Total path distance: {min_distance:.2f} meters")

# ── Upload to Pixhawk ───────────────────────────────────────
print(" Uploading waypoints...")
vehicle.commands.clear()
vehicle.commands.upload()
time.sleep(1)

for lat, lon, alt in best_waypoints:
    cmd = Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1,
        delay_seconds, 0, 0, 0,
        lat, lon, alt
    )
    vehicle.commands.add(cmd)

vehicle.commands.upload()
vehicle.commands.download()
vehicle.commands.wait_ready()

print(f" {len(best_waypoints)} waypoints with {delay_seconds}s delay uploaded.")

# ── Optional plot ───────────────────────────────────────────
x, y = zip(*best_path)
plt.figure(figsize=(10, 8))
plt.plot(x, y, marker='o', label=f'Zigzag ({best_angle}°)')
plt.scatter(x[-1], y[-1], color='red', label='Final Waypoint')
plt.plot(*polygon.exterior.xy, 'k--', alpha=0.5, label='Geofence')
plt.title(f'Zigzag Coverage Path (Delay = {delay_seconds}s)')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

vehicle.close()
print(" Disconnected.")


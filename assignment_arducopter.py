import time
import numpy as np
from pymavlink import mavutil

# Connect to ArduCopter SITL via TCP port 5762 (SERIAL1)
the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
the_connection.wait_heartbeat()
print("Connected to ArduCopter SITL!")

# 1. Switch to GUIDED mode (Mode 4)
print("Setting GUIDED mode...")
the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4
)
time.sleep(1)

# 2. Arm the vehicle
print("Arming vehicle...")
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print("Arm ACK:", msg)
time.sleep(2)

# 3. Takeoff to 20 meters altitude
print("Taking off to 20 meters...")
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 20
)

while True:
    hud = the_connection.recv_match(type='VFR_HUD', blocking=True)
    if hud and hud.alt > 18:
        print(f"Reached takeoff altitude: {hud.alt:.1f}m")
        break

gps = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
centre_lat = gps.lat / 1e7
centre_lon = gps.lon / 1e7

# 4. Generate 200x100m Lawnmower Grid with 20m Spacing
def generate_lawnmower_grid(c_lat, c_lon):
    R = 6371000
    length = 200.0
    width = 100.0
    spacing = 20.0
    
    waypoints = []
    y_coords = np.arange(0, width + 1, spacing)
    
    for i, y in enumerate(y_coords):
        x_coords = [0, length] if i % 2 == 0 else [length, 0]
        for x in x_coords:
            lat = c_lat + (y / R) * (180 / np.pi)
            lon = c_lon + (x / (R * np.cos(c_lat * np.pi / 180))) * (180 / np.pi)
            waypoints.append((lat, lon))
            
    return waypoints

all_waypoints = generate_lawnmower_grid(centre_lat, centre_lon)
print(f"Generated {len(all_waypoints)} waypoints.")

FLIGHT_1_LIMIT = len(all_waypoints) // 2

def execute_flight_segment(start_idx, end_idx, flight_name):
    print(f"\n--- {flight_name} (WP {start_idx} to {end_idx}) ---")
    for i in range(start_idx, end_idx):
        lat, lon = all_waypoints[i]
        the_connection.mav.set_position_target_global_int_send(
            0,
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            20, 0, 0, 0, 0, 0, 0, 0, 0
        )
        
        start_time = time.time()
        while True:
            if time.time() - start_time > 45:
                break
            g = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if not g:
                continue
            if abs(g.lat / 1e7 - lat) < 0.00015 and abs(g.lon / 1e7 - lon) < 0.00015:
                print(f"Reached WP{i}")
                break
            time.sleep(0.5)

# Flight 1
execute_flight_segment(0, FLIGHT_1_LIMIT, "FLIGHT 1")

# RTL for battery simulation
print("\n[BATTERY WARNING] Triggering RTL...")
the_connection.mav.set_mode_send(the_connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
input(">>> Press ENTER once battery is 'swapped/reset': ")

# Relaunch Flight 2
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(2)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 20)
time.sleep(10)

# Resume Flight 2
execute_flight_segment(FLIGHT_1_LIMIT, len(all_waypoints), "FLIGHT 2 (RESUMED)")

# Land
the_connection.mav.set_mode_send(the_connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
print("Mission Complete!")
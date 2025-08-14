from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from fastapi.middleware.cors import CORSMiddleware
from pymavlink import mavutil
import threading

# Initialize FastAPI app
app = FastAPI()

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Connect to the drone via DroneKit
try:
    connection_string = "/dev/ttyUSB0"  # Update to your FC port if using USB
    vehicle = connect(connection_string, baud=57600, wait_ready=True, heartbeat_timeout=60)
    print("Drone connected successfully via DroneKit.")
except Exception as e:
    print("Failed to connect to the drone:", e)
    vehicle = None

# --- Optional: also connect via pymavlink directly ---
try:
    mav = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)
    mav.wait_heartbeat()
    print(f"Heartbeat received from system {mav.target_system}")
except Exception as e:
    print("Failed to connect via MAVLink:", e)
    mav = None

# Data models
class Coordinates(BaseModel):
    latitude: float
    longitude: float
    altitude: float = None  # Optional

class RectangleBounds(BaseModel):
    top_left: Coordinates
    bottom_right: Coordinates

latest_coords: Coordinates | None = None

# --- MAVLink message handler ---
def handle_mavlink_coordinate(lat, lon, alt):
    global vehicle
    if vehicle is not None:
        target_location = LocationGlobalRelative(lat, lon, alt)
        print(f"[Backend] simple_goto -> {target_location}")
        vehicle.simple_goto(target_location)
    else:
        print("[Backend] Vehicle not connected; skipping goto.")

# Example: function to read from pymavlink (optional)
def mavlink_listener():
    global mav
    while mav is not None:
        msg = mav.recv_match(type='SET_POSITION_TARGET_GLOBAL_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt
            print(f"[MAVLink] Received lat={lat}, lon={lon}, alt={alt}")
            handle_mavlink_coordinate(lat, lon, alt)

# Start MAVLink listener thread
if mav is not None:
    t = threading.Thread(target=mavlink_listener, daemon=True)
    t.start()

# --- DroneKit flight functions ---
def arm_and_takeoff(target_altitude):
    if vehicle is None:
        raise HTTPException(status_code=500, detail="Drone not connected")

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Utility: distance in meters
def get_distance_meters(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return ((dlat**2) + (dlong**2)) ** 0.5 * 1.113195e5

# Rectangle movement within bounds
def move_within_rectangle(top_left, bottom_right, altitude):
    if vehicle is None:
        raise HTTPException(status_code=500, detail="Drone not connected")

    step_distance = 0.0002
    current_lat = top_left.latitude
    current_lon = top_left.longitude
    moving_forward = True
    max_retry = 10
    retry_count = 0

    while current_lat >= bottom_right.latitude:
        if moving_forward:
            while current_lon < bottom_right.longitude:
                target_location = LocationGlobalRelative(current_lat, current_lon, altitude)
                vehicle.simple_goto(target_location)
                retry_count = 0
                while True:
                    distance = get_distance_meters(vehicle.location.global_relative_frame, target_location)
                    if distance < 5:
                        break
                    if retry_count >= max_retry:
                        vehicle.simple_goto(target_location)
                        retry_count = 0
                    retry_count += 1
                    time.sleep(1)
                current_lon += step_distance
        else:
            while current_lon > top_left.longitude:
                target_location = LocationGlobalRelative(current_lat, current_lon, altitude)
                vehicle.simple_goto(target_location)
                retry_count = 0
                while True:
                    distance = get_distance_meters(vehicle.location.global_relative_frame, target_location)
                    if distance < 5:
                        break
                    if retry_count >= max_retry:
                        vehicle.simple_goto(target_location)
                        retry_count = 0
                    retry_count += 1
                    time.sleep(1)
                current_lon -= step_distance
        if current_lat > bottom_right.latitude:
            current_lat -= step_distance
        else:
            current_lat += step_distance
        moving_forward = not moving_forward

# --- FastAPI endpoint ---
@app.post("/drone/dispatch/rectangle")
async def dispatch_drone_rectangle(bounds: RectangleBounds):
    if vehicle is None:
        raise HTTPException(status_code=500, detail="Drone not connected")

    top_left = bounds.top_left
    bottom_right = bounds.bottom_right

    arm_and_takeoff(10)
    move_within_rectangle(top_left, bottom_right, 10)

    print("Returning to home position")
    vehicle.mode = VehicleMode("RTL")
    while not vehicle.location.global_relative_frame.alt <= 1:
        time.sleep(1)

    vehicle.close()
    return {"status": "Drone dispatched and returned home."}

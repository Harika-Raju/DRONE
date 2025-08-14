import torch
import cv2
import os
import random
import math
from pymavlink import mavutil
import time

# Function to calculate the change in latitude and longitude based on distance
def distance_to_lat_lon(distance, altitude, latitude, longitude, frame_width):
    meters_per_degree_lat = 111320
    meters_per_degree_lon = 40008000 / 360
    horizontal_fov = 2 * altitude * math.tan(math.radians(45) / 2)
    pixel_size_meters = horizontal_fov / frame_width
    distance_degrees_lat = (distance * pixel_size_meters) / meters_per_degree_lat
    distance_degrees_lon = (distance * pixel_size_meters) / meters_per_degree_lon
    new_latitude = latitude + distance_degrees_lat
    new_longitude = longitude + distance_degrees_lon
    return new_latitude, new_longitude

# Function to calculate distance from bounding box
def calculate_distance(bbox, altitude, frame_width, frame_height, fov=45):
    x_min, y_min, x_max, y_max = bbox[:4]
    bbox_center_x = (x_min + x_max) / 2
    bbox_center_y = (y_min + y_max) / 2
    fov_rad = math.radians(fov)
    horizontal_fov_meters = 2 * altitude * math.tan(fov_rad / 2)
    pixel_size_meters = horizontal_fov_meters / frame_width
    distance = ((bbox_center_x - frame_width / 2) ** 2 + (bbox_center_y - frame_height / 2) ** 2) ** 0.5
    return distance * pixel_size_meters

# --- MAVLink setup ---
# Connect to flight controller via serial (adjust port and baud as needed)
mav = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
mav.wait_heartbeat()
print(f"Heartbeat received from system {mav.target_system}")

# Function to send latitude/longitude via MAVLink
def send_mavlink_coordinate(lat, lon, alt):
    try:
        # Send a SET_POSITION_TARGET_GLOBAL_INT message
        mav.mav.set_position_target_global_int_send(
            int(time.time() * 1e6),  # time_boot_ms
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # type_mask: only positions enabled
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0
        )
        print(f"[MAVLink] Sent -> lat: {lat}, lon: {lon}, alt: {alt}")
    except Exception as e:
        print(f"[MAVLink] Send error: {e}")

# Function to run YOLO detection
def run_video_detection_with_mavlink(weights_path, altitude, initial_latitude, initial_longitude, output_dir="output", conf_threshold=0.25):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    os.makedirs(output_dir, exist_ok=True)
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
    model.conf = conf_threshold
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        frame_height, frame_width = frame.shape[:2]
        results = model(frame)
        detections = results.xyxy[0].cpu().numpy()

        if len(detections) > 0:
            random_detection = random.choice(detections)
            distance = calculate_distance(random_detection, altitude, frame_width, frame_height)
            new_lat, new_lon = distance_to_lat_lon(distance, altitude, initial_latitude, initial_longitude, frame_width)
            print(f"Detection -> lat: {new_lat:.6f}, lon: {new_lon:.6f}")

            # --- Send via MAVLink ---
            send_mavlink_coordinate(new_lat, new_lon, altitude)

            annotated_frame = results.render()[0]
            output_path = os.path.join(output_dir, f"detection_frame_{frame_count}.jpg")
            cv2.imwrite(output_path, annotated_frame)

            break  # exit after one detection for testing

        annotated_frame = results.render()[0]
        cv2.imshow("Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# --- Run detection ---
if __name__ == "__main__":
    weights_path = "yolov5/weights/best.pt"
    altitude = 5.0
    initial_latitude = 12.9716
    initial_longitude = 77.5946
    output_dir = "output"
    run_video_detection_with_mavlink(weights_path, altitude, initial_latitude, initial_longitude, output_dir)

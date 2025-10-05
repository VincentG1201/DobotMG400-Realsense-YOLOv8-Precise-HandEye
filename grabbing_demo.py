import cv2
import numpy as np
import pyrealsense2 as rs
from time import sleep
from ultralytics import YOLO
import sys
import os

# Import shared settings and robot connection function.
from calibration_config import *
summer_project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, summer_project_dir)
from dobot_connect import connect

# --- Helper Functions ---
def process_frame(frame, model, confidence_threshold=0.65):
    # Runs the YOLO model on a single image frame.
    results = model(frame, conf=confidence_threshold)
    return results[0]

def get_all_objects(result):
    # Returns a list of all detected object boxes.
    if hasattr(result, 'obb') and result.obb.boxes:
        return result.obb.boxes
    return []

def convert_pixel_to_robot_coords(pixel_point, H):
    # Converts a single pixel point (u, v) to robot coordinates (x, y).
    pixel_u, pixel_v = pixel_point
    pixel_vector = np.array([[pixel_u], [pixel_v], [1]], dtype=np.float32)
    robot_coords_homogeneous = H @ pixel_vector
    x_robot = robot_coords_homogeneous[0] / robot_coords_homogeneous[2]
    y_robot = robot_coords_homogeneous[1] / robot_coords_homogeneous[2]
    return x_robot[0], y_robot[0]

def run_demo():
    # ------------------- USER ACTION REQUIRED -------------------
    # PLEASE SET THIS VALUE CAREFULLY.
    # This is the Z-axis height (in mm) where the robot will attempt to grab objects.
    # If this is too low, the robot may collide with the table, causing an alarm.
    # If this is too high, it will not be able to pick up the object.
    # A good starting value is usually between -40 and -50.
    GRABBING_HEIGHT = -45.0
    # -----------------------------------------------------------

    HOVERING_HEIGHT_OFFSET = 20.0 # How many mm to lift the object after grabbing.
    
    print("--- Dobot Pick and Place Demo ---")
    print("!!! WARNING !!!")
    print(f"The robot is set to grab objects at a height of Z = {GRABBING_HEIGHT} mm.")
    print("Ensure this height is safe for your workspace before proceeding.")
    input("Press Enter to continue if the height is set correctly...")

    # --- Load Calibration Data ---
    try:
        homography_matrix = np.loadtxt(HOMOGRAPHY_MATRIX_FILE, delimiter=",")
        # Load all four values from the final calibration file.
        calib_data = np.loadtxt(FINAL_CALIBRATION_FILE, delimiter=",")
        work_offset_x = calib_data[2]
        work_offset_y = calib_data[3]
        print("All calibration files loaded successfully.")
    except FileNotFoundError:
        print("Error: Calibration files not found.")
        print("Please run both stage_1 and stage_2 calibration scripts first.")
        return

    # --- Initialization ---
    move, dashboard, _ = connect()
    if not move: return
    dashboard.EnableRobot()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    model = YOLO(YOLO_MODEL_PATH)

    try:
        while True:
            # --- Move to the fixed calibration pose to see the objects ---
            print("\nMoving to observation position...")
            dashboard.AccJ(20) # Use a normal acceleration for general travel.
            move.MovJ(CALIBRATION_POSE['x'], CALIBRATION_POSE['y'], CALIBRATION_POSE['z'], CALIBRATION_POSE['r'])
            move.Sync()
            sleep(2)

            # --- Detect all objects in the scene ---
            frames = pipeline.wait_for_frames()
            image = np.asanyarray(align.process(frames).get_color_frame().get_data())
            result = process_frame(image, model)
            all_objects = get_all_objects(result)

            if not all_objects:
                print("No objects found. Please place objects in the workspace.")
            else:
                print(f"Found {len(all_objects)} objects. Starting pick and place routine.")
                
                for i, box in enumerate(all_objects):
                    print(f"\n--- Processing object {i+1}/{len(all_objects)} ---")
                    
                    pixel_x, pixel_y = int(box.xywhr[0][0]), int(box.xywhr[0][1])
                    
                    # Calculate the precise robot coordinates for the object's center.
                    robot_x_initial, robot_y_initial = convert_pixel_to_robot_coords((pixel_x, pixel_y), homography_matrix)
                    
                    # Apply the camera-to-gripper offset to get the gripper's target coordinates.
                    final_x = robot_x_initial + work_offset_x
                    final_y = robot_y_initial + work_offset_y
                    
                    print(f"Target coordinates: (X: {final_x:.2f}, Y: {final_y:.2f})")

                    # --- Execute the pick and place sequence ---
                    # 1. Move to a hovering position above the object with normal acceleration.
                    dashboard.AccJ(20)
                    move.MovJ(final_x, final_y, GRABBING_HEIGHT + HOVERING_HEIGHT_OFFSET, CALIBRATION_POSE['r'])
                    move.Sync()
                    
                    # 2. Open gripper (DO Port 1, Value 0 = Off)
                    dashboard.DO(1, 0)
                    sleep(1)

                    # 3. SET LOW ACCELERATION for the final, careful approach.
                    print("Slowing down for final approach...")
                    dashboard.AccL(5) # Use AccL for linear moves. A small value like 5 is gentle.
                    
                    # 4. Move down to the grabbing height.
                    move.MovL(final_x, final_y, GRABBING_HEIGHT, CALIBRATION_POSE['r'])
                    move.Sync()
                    
                    # 5. Close gripper to grab the object (DO Port 1, Value 1 = On)
                    dashboard.DO(1, 1)
                    sleep(1)
                    
                    # 6. Lift the object up slowly.
                    move.MovL(final_x, final_y, GRABBING_HEIGHT + HOVERING_HEIGHT_OFFSET, CALIBRATION_POSE['r'])
                    move.Sync()

                    # 7. RESET ACCELERATION to normal for faster general movement.
                    dashboard.AccL(20)

                    # 8. Place it back down.
                    move.MovL(final_x, final_y, GRABBING_HEIGHT, CALIBRATION_POSE['r'])
                    move.Sync()
                    
                    # 9. Open gripper to release the object.
                    dashboard.DO(1, 0)
                    sleep(1)

                    # 10. Move back up to the hovering height.
                    move.MovL(final_x, final_y, GRABBING_HEIGHT + HOVERING_HEIGHT_OFFSET, CALIBRATION_POSE['r'])
                    move.Sync()

            user_input = input("\nDemo cycle complete. Run again? (y/n): ").lower()
            if user_input != 'y':
                break

    finally:
        # --- Cleanup ---
        print("Disabling robot and shutting down camera.")
        dashboard.DisableRobot()
        pipeline.stop()

if __name__ == "__main__":
    run_demo()
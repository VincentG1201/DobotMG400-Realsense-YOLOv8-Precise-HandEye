import cv2
import numpy as np
import pyrealsense2 as rs
from time import sleep
from ultralytics import YOLO
import sys
import os

# Import shared settings and robot connection functions.
from calibration_config import *
summer_project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, summer_project_dir)
from dobot_connect import connect

# --- Helper Functions ---
def process_frame(frame, model, confidence_threshold=0.65):
    results = model(frame, conf=confidence_threshold)
    return results[0]

def get_all_objects(result):
    if hasattr(result, 'obb') and result.obb.boxes:
        return result.obb.boxes
    return []

def get_target_object(result):
    IMAGE_CENTER_X = IMAGE_WIDTH // 2
    IMAGE_CENTER_Y = IMAGE_HEIGHT // 2
    min_dist = float('inf')
    target_box = None
    for box in get_all_objects(result):
        px, py = int(box.xywhr[0][0]), int(box.xywhr[0][1])
        dist = np.sqrt((px - IMAGE_CENTER_X)**2 + (py - IMAGE_CENTER_Y)**2)
        if dist < min_dist:
            min_dist = dist
            target_box = box
    return target_box

def convert_pixel_to_robot_coords(pixel_point, H):
    pixel_u, pixel_v = pixel_point
    pixel_vector = np.array([[pixel_u], [pixel_v], [1]], dtype=np.float32)
    robot_coords_homogeneous = H @ pixel_vector
    x_robot = robot_coords_homogeneous[0] / robot_coords_homogeneous[2]
    y_robot = robot_coords_homogeneous[1] / robot_coords_homogeneous[2]
    return x_robot[0], y_robot[0]

# --- Main Calibration Steps for Stage 2 ---

def step_1_calculate_avg_mm_per_pixel():
    """
    Iteratively calculates the mm/pixel ratio for each detected object,
    averages the results, and saves them to a file.
    """
    print("Executing Step 1: Calculating average mm/pixel ratio.")
    
    # Initialization
    move, dashboard, feedback = connect()
    if not move: return
    dashboard.EnableRobot()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    model = YOLO(YOLO_MODEL_PATH)
    homography_matrix = np.loadtxt(HOMOGRAPHY_MATRIX_FILE, delimiter=",")
    
    initial_mm_per_pixel_x = 0.46
    initial_mm_per_pixel_y = 0.43

    move.MovJ(CALIBRATION_POSE['x'], CALIBRATION_POSE['y'], CALIBRATION_POSE['z'], CALIBRATION_POSE['r'])
    move.Sync()
    sleep(2)

    frames = pipeline.wait_for_frames()
    image = np.asanyarray(align.process(frames).get_color_frame().get_data())
    all_objects = get_all_objects(process_frame(image, model))

    if not all_objects:
        print("Error: No objects found.")
        return

    print(f"Found {len(all_objects)} objects for calibration.")
    mm_per_pixel_x_values = []
    mm_per_pixel_y_values = []

    for i, box in enumerate(all_objects):
        print(f"\n--- Processing object {i+1}/{len(all_objects)} ---")
        
        initial_px, initial_py = int(box.xywhr[0][0]), int(box.xywhr[0][1])
        target_x, target_y = convert_pixel_to_robot_coords((initial_px, initial_py), homography_matrix)
        
        move.MovJ(target_x, target_y, -35.0, 0)
        move.Sync()
        sleep(2)

        frames = pipeline.wait_for_frames()
        image_after_move = np.asanyarray(align.process(frames).get_color_frame().get_data())
        current_box = get_target_object(process_frame(image_after_move, model))
        if not current_box: continue

        px_before_correction = int(current_box.xywhr[0][0])
        py_before_correction = int(current_box.xywhr[0][1])
        
        move_x_mm = (IMAGE_WIDTH // 2 - px_before_correction) * initial_mm_per_pixel_x
        move_y_mm = (IMAGE_HEIGHT // 2 - py_before_correction) * initial_mm_per_pixel_y
        
        move.MovJ(target_x + move_x_mm, target_y + move_y_mm, -35.0, 0)
        move.Sync()
        sleep(2)

        frames = pipeline.wait_for_frames()
        final_image = np.asanyarray(align.process(frames).get_color_frame().get_data())
        final_box = get_target_object(process_frame(final_image, model))
        if not final_box: continue

        final_px = int(final_box.xywhr[0][0])
        final_py = int(final_box.xywhr[0][1])

        if abs(final_px - px_before_correction) > 0:
            real_mm_per_pixel_x = (abs(IMAGE_WIDTH // 2 - px_before_correction) * initial_mm_per_pixel_x) / abs(final_px - px_before_correction)
            mm_per_pixel_x_values.append(real_mm_per_pixel_x)

        if abs(final_py - py_before_correction) > 0:
            real_mm_per_pixel_y = (abs(IMAGE_HEIGHT // 2 - py_before_correction) * initial_mm_per_pixel_y) / abs(final_py - py_before_correction)
            mm_per_pixel_y_values.append(real_mm_per_pixel_y)

        move.MovJ(CALIBRATION_POSE['x'], CALIBRATION_POSE['y'], CALIBRATION_POSE['z'], CALIBRATION_POSE['r'])
        move.Sync()

    if mm_per_pixel_x_values and mm_per_pixel_y_values:
        average_x = np.mean(mm_per_pixel_x_values)
        average_y = np.mean(mm_per_pixel_y_values)
        np.savetxt(MM_PER_PIXEL_FILE, [average_x, average_y], delimiter=",")
        print(f"\nStep 1 Complete. Average mm/pixel values saved to {MM_PER_PIXEL_FILE}")
    else:
        print("Step 1 Failed. Could not calculate valid ratios.")

    dashboard.DisableRobot()
    pipeline.stop()

def step_2_center_camera_on_object():
    """
    Uses the calculated mm/pixel ratio to center the camera over an object
    and saves the robot's theoretical position.
    """
    print("\nExecuting Step 2: Centering camera on an object.")
    
    # Initialization
    move, dashboard, feedback = connect()
    if not move: return
    dashboard.EnableRobot()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    model = YOLO(YOLO_MODEL_PATH)
    mm_per_pixel_x, mm_per_pixel_y = np.loadtxt(MM_PER_PIXEL_FILE, delimiter=",")
    
    move.MovJ(CALIBRATION_POSE['x'], CALIBRATION_POSE['y'], CALIBRATION_POSE['z'], CALIBRATION_POSE['r'])
    move.Sync()

    input("Place a single target object in view, then press Enter...")
    
    frames = pipeline.wait_for_frames()
    image = np.asanyarray(align.process(frames).get_color_frame().get_data())
    target_box = get_target_object(process_frame(image, model))
    
    if not target_box:
        print("Error: No object found.")
        return

    px = int(target_box.xywhr[0][0])
    py = int(target_box.xywhr[0][1])
    
    move_x_mm = (IMAGE_WIDTH // 2 - px) * mm_per_pixel_x
    move_y_mm = (IMAGE_HEIGHT // 2 - py) * mm_per_pixel_y

    new_x = CALIBRATION_POSE['x'] + move_x_mm
    new_y = CALIBRATION_POSE['y'] + move_y_mm
    
    move.MovJ(new_x, new_y, CALIBRATION_POSE['z'], CALIBRATION_POSE['r'])
    move.Sync()

    np.savetxt(CAMERA_CENTERED_POSE_FILE, [new_x, new_y, CALIBRATION_POSE['z'], CALIBRATION_POSE['r']], delimiter=",")
    print(f"Step 2 Complete. Camera-centered pose saved to {CAMERA_CENTERED_POSE_FILE}")
    
    dashboard.DisableRobot()
    pipeline.stop()

def step_3_get_manual_work_offset():
    """
    Calculates and saves the real-world X/Y offset between the camera
    center and the gripper center based on manual user input.
    """
    print("\nExecuting Step 3: Finding the End-Effector Offset.")
    
    camera_centered_pose = np.loadtxt(CAMERA_CENTERED_POSE_FILE, delimiter=",")
    
    print("The robot is currently where the camera was centered.")
    print(f"Theoretical Position is X: {camera_centered_pose[0]:.2f}, Y: {camera_centered_pose[1]:.2f}")
    print("Please MANUALLY JOG the robot until the GRIPPER is centered on the same object.")
    
    try:
        final_x = float(input("Enter the final X coordinate from the robot's screen: "))
        final_y = float(input("Enter the final Y coordinate from the robot's screen: "))
    except ValueError:
        print("Invalid input. Please enter numbers only.")
        return

    offset_x = final_x - camera_centered_pose[0]
    offset_y = final_y - camera_centered_pose[1]

    np.savetxt(WORK_OFFSET_FILE, [offset_x, offset_y], delimiter=",")
    print(f"Step 3 Complete. Work offset saved to {WORK_OFFSET_FILE}")
    
    # Final step: Combine all data into one file.
    mm_px = np.loadtxt(MM_PER_PIXEL_FILE, delimiter=",")
    final_data = [mm_px[0], mm_px[1], offset_x, offset_y]
    np.savetxt(FINAL_CALIBRATION_FILE, final_data, delimiter=",")
    print(f"\nSuccess! All calibration data combined and saved to {FINAL_CALIBRATION_FILE}")


if __name__ == "__main__":
    
    pass
    # --- Instructions ---
    # Uncomment and run each function one by one.
    
    # 1. Run this first to get the mm/pixel ratio.
    # step_1_calculate_avg_mm_per_pixel()
    
    # 2. Run this second to center the camera and record its pose.
    # step_2_center_camera_on_object()
    
    # 3. Run this last. Manually center the gripper and input its coordinates.
    # step_3_get_manual_work_offset()
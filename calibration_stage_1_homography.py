import cv2
import numpy as np
import pyrealsense2 as rs
from time import sleep
import sys
import os

# Import shared settings and robot connection function.
from calibration_config import *
summer_project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, summer_project_dir)
from dobot_function.dobot_connect import connect

def step_1_detect_corners():
    """
    Moves the robot to the fixed pose, captures a chessboard image,
    and saves the pixel coordinates of the corners to a file.
    """
    move, dashboard, _ = connect()
    if not move or not dashboard:
        return
    dashboard.EnableRobot()

    print("Step 1: Detecting Chessboard Corners")
    print(f"Moving to fixed calibration pose: {CALIBRATION_POSE}")
    move.MovJ(CALIBRATION_POSE['x'], CALIBRATION_POSE['y'], CALIBRATION_POSE['z'], CALIBRATION_POSE['r'])
    sleep(5)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
    pipeline.start(config)

    print("Please place chessboard in view and press 's' to save.")
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("Chessboard Capture", color_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                np.savetxt(PIXEL_POINTS_FILE, np.squeeze(corners_subpix), delimiter=",")
                print(f"Success! Pixel points saved to {PIXEL_POINTS_FILE}")
                break
            else:
                print("Chessboard not found. Please try again.")
        elif key == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()
    dashboard.DisableRobot()
    print("Step 1 complete.")

def step_2_get_robot_poses():
    """
    This function holds the manually entered robot coordinates.
    It returns the numpy array directly.
    """
    print("Step 2: Getting Robot Poses")
    
    # Manually move the robot arm to each corner corresponding to the pixel points.
    # Record the [X, Y] coordinates from the robot for each point.
    # Paste the coordinates directly into this array.
    robot_poses_mm = np.array([
        [234.57, -93.81], [234.73, -68.66], [235.90, -43.36], [237.21, -17.88], [237.80, 7.50], [237.95, 33.13], [239.21, 57.89],
        [259.23, -95.47], [260.06, -69.34], [260.80, -43.74], [261.92, -18.74], [262.33, 6.69], [264.03, 31.94], [264.37, 57.38],
        [285.67, -96.79], [286.01, -71.07], [286.81, -45.28], [286.93, -18.87], [288.34, 5.36], [288.96, 31.04], [289.62, 56.42],
        [309.94, -96.12], [310.84, -71.54], [311.78, -46.22], [313.24, -21.02], [313.81, 4.68], [315.14, 29.73], [315.34, 55.41],
        [335.81, -98.01], [336.38, -72.69], [337.40, -47.64], [338.30, -21.76], [339.48, 3.34], [339.90, 28.72], [341.22, 53.64],
        [360.75, -98.42], [361.68, -73.49], [362.53, -47.96], [363.38, -22.76], [364.60, 2.72], [365.34, 28.14], [366.01, 53.66],
        [385.78, -99.20], [386.27, -74.12], [387.32, -49.07], [388.58, -23.59], [389.52, 2.21], [390.48, 27.23], [391.56, 52.88]
    ])
    
    print("Robot poses are ready.")
    return robot_poses_mm

def step_3_calculate_homography(robot_poses_mm):
    """
    Loads the pixel points, takes the robot poses as input,
    and calculates the final homography matrix.
    """
    print("Step 3: Calculating Homography Matrix")
    
    try:
        pixel_points = np.loadtxt(PIXEL_POINTS_FILE, delimiter=",")
    except FileNotFoundError:
        print(f"Error: Could not find pixel points file: {PIXEL_POINTS_FILE}")
        print("Please ensure you have run Step 1 successfully.")
        return

    if len(pixel_points) != len(robot_poses_mm):
        print("Error: The number of pixel points and robot poses do not match.")
        return

    homography_matrix, _ = cv2.findHomography(pixel_points, robot_poses_mm)
    np.savetxt(HOMOGRAPHY_MATRIX_FILE, homography_matrix, delimiter=",")
    
    print(f"Homography matrix saved to {HOMOGRAPHY_MATRIX_FILE}")
    print("Stage 1 is complete.")

if __name__ == "__main__":
    # --- Instructions ---
    # 1. Run the script once to execute step_1_detect_corners().
    # 2. After it completes, edit the array in step_2_get_robot_poses() with your data.
    # 3. Comment out step_1 and uncomment the other two lines to finish the process.

    # step_1_detect_corners()
    
    robot_poses = step_2_get_robot_poses()
    step_3_calculate_homography(robot_poses)
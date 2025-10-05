# -------------------
# Shared Settings for Calibration
# -------------------

# This is the single, fixed robot pose for capturing images.
# It is critical that this pose is not changed between Stage 1 and Stage 2.
CALIBRATION_POSE = {
    "x": 259.96,
    "y": -7.78,
    "z": 160.00,
    "r": 0.00,
}

# --- File Paths ---
# This section defines all the data files used and created during calibration.
##MODIFY!!!!
DATA_PATH = r"C:\Users\XilinGuan\Desktop\summer_project"

# Stage 1 Files
PIXEL_POINTS_FILE = DATA_PATH + "\\pixel_points.txt" # Stores chessboard corner coordinates in pixels.
HOMOGRAPHY_MATRIX_FILE = DATA_PATH + "\\homography_matrix.txt" # Stores the calculated transformation matrix.

# Stage 2 Intermediate Files
MM_PER_PIXEL_FILE = DATA_PATH + "\\mm_per_pixel.txt" # Stores the calculated mm/pixel ratio.
CAMERA_CENTERED_POSE_FILE = DATA_PATH + "\\camera_centered_pose.txt" # Stores the robot pose when the camera is centered.
WORK_OFFSET_FILE = DATA_PATH + "\\work_offset.txt" # Stores the manually found gripper-to-camera offset.

# Final Combined Output File
FINAL_CALIBRATION_FILE = DATA_PATH + "\\final_calibration_data.txt" # Stores the final, combined calibration results.

# Path to your trained YOLO model.
##MODIFY!!!!
YOLO_MODEL_PATH = r"C:\Users\XilinGuan\Desktop\summer_project\YOLO\runs\obb\train\weights\best.pt"

# Camera and Chessboard settings.
CHESSBOARD_SIZE = (7, 7)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
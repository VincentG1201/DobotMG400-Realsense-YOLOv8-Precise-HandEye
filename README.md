# DobotMG400-Realsense-YOLOv8-Precise-HandEye
# Vision-Guided Robotic Arm Calibration

This project implements a **two-stage vision–robot calibration pipeline** that enables a robotic arm to accurately convert camera pixel coordinates into real-world robot coordinates and align its gripper with detected objects. The system is designed for **vision-based positioning and grasping**, combining classical computer vision, deep learning, and practical robot calibration techniques.

---

## 🚀 Project Overview

In real-world robotic manipulation, directly mapping image pixels to robot coordinates is challenging due to camera mounting errors, perspective distortion, and end-effector offsets. This project addresses those challenges through a **structured, modular calibration process**:

- **Stage 1**: Establish a geometric mapping from image pixels to robot base coordinates using a chessboard and homography.
- **Stage 2**: Refine the mapping using object detection (YOLO), estimate real-world scale (mm/pixel), and compute the offset between the camera and the gripper.

The result is a practical calibration pipeline that allows the robot to **detect an object in the image and move its gripper accurately to that object in the real world**.

---

## 🧠 System Architecture

**Hardware**
- Dobot robotic arm  
- Intel RealSense RGB camera (mounted on the arm)  
- Chessboard calibration target  

**Software**
- Python  
- OpenCV  
- NumPy  
- Intel RealSense SDK (`pyrealsense2`)  
- Ultralytics YOLO (OBB model)  

---

## 📐 Calibration Pipeline

### Stage 1: Pixel-to-Robot Mapping (Homography)

**Goal:** Compute a transformation that maps 2D image pixel coordinates to robot base coordinates.

1. **Fixed-Pose Image Capture**  
   The robot moves to a single, fixed calibration pose. An image of a chessboard is captured, and chessboard corner pixel coordinates are detected with sub-pixel accuracy.

2. **Manual Robot Correspondence Collection**  
   The robot is manually jogged so that the gripper aligns with each chessboard corner. The corresponding robot **(X, Y)** coordinates are recorded.

3. **Homography Calculation**  
   Using the pixel–robot point pairs, a homography matrix is computed and saved. This matrix provides a **coarse pixel → robot coordinate mapping**.

---

### Stage 2: Real-World Refinement

Stage 1 provides a geometric mapping, but real-world factors such as scale distortion and camera–gripper misalignment remain. Stage 2 refines the calibration.

1. **mm-per-Pixel Estimation**  
   - Objects are detected using a trained YOLO OBB model.  
   - The robot moves toward detected objects using the homography result.  
   - Pixel displacements from the image center are measured before and after motion.  
   - Separate **mm/pixel ratios for X and Y** are calculated and averaged.

2. **Camera Centering Calibration**  
   Using the refined mm/pixel values, the robot moves so that the **camera optical center** is aligned with a detected object. The corresponding robot pose is saved.

3. **Gripper Offset Calibration**  
   The user manually jogs the robot so the **gripper**, rather than the camera, is centered on the same object. The offset between the camera-centered pose and the gripper-centered pose is computed and saved.

---

## ▶️ How to Use This Repository

This guide walks you through the **complete workflow** for using this calibration system from scratch.  
It explains **what to modify first**, **when to re-run each step**, and **where to update files when new data is collected**.

---

## 1️⃣ Initial Setup (Do This First)

### Modify `calibration_config.py`

This file is the **central configuration file** for the entire project.  
Before running any scripts, open `calibration_config.py` and update:

- **`DATA_PATH`**  
  Set this to the directory where all calibration data will be saved.

- **`YOLO_MODEL_PATH`**  
  Set this to the path of your trained YOLO model (`best.pt`).

- **`CALIBRATION_POSE`**  
  This is the fixed robot pose used during calibration.  
  ⚠️ **Do not change this pose between Stage 1 and Stage 2.**

- **Camera & chessboard settings**  
  Update `IMAGE_WIDTH`, `IMAGE_HEIGHT`, and `CHESSBOARD_SIZE` if your setup differs.

> ✅ If you move the camera, change the robot, or change the workspace, re-check this file first.

---

## 2️⃣ Stage 1 — Homography Calibration (Pixel → Robot)

### Step 1: Capture Chessboard Pixel Points

# 1. Place the chessboard flat on the working surface.
# 2. Run `calibration_stage_1_homography.py`.
# 3. Uncomment and run:
   ```python
   step_1_detect_corners()
   ```
Press s when the chessboard is detected.

### Step 2 — Update Robot Coordinate Data (Manual)
# 1. Manually jog the robot to align the gripper with each chessboard corner
# 2. Record robot X/Y coordinates
# 3. Open calibration_stage_1_homography.py
# 4. Replace the values inside robot_poses_mm in:
    ```python
    step_2_get_robot_poses()
    ```

### Step 3 — Compute Homography Matrix
# 1. Comment out step_1_detect_corners()
# 2. Run:
    ```python
    robot_poses = step_2_get_robot_poses()
    step_3_calculate_homography(robot_poses)
    ```
# output
homographt_matrix.txt

## 3️⃣ Stage 2: Calibration Refinement
### Step 1 — Estimate mm-per-Pixel
# 1. Place multiple detectable objects in the workspace
# 2. In calibration_stage_2_refinement.py, uncomment:
    ```python
    step_1_calculate_avg_mm_per_pixel()
    ```
# 3. Run the script
# output
mm_per_pixel.txt
## 🔁 Re-run if camera height or lens changes.


### Step 2 — Center Camera on Object
# 1. Place one object in the workspace
# 2. Uncomment:
    ```python
    step_2_center_camera_on_object()
    ```
# 3. Run the script and follow the prompt
# output
camera_centered_pose.txt


### Step 3 — Compute Gripper Offset (Manual)
# 1. Uncomment:
    ```python   
    step_3_get_manual_work_offset()
    ```
# 2. Run the script
# 3. Manually jog the robot until the gripper is centered on the object
# 4. Enter final robot X/Y coordinates when prompted
# output
work_offset.txt
final_calibration_data.txt

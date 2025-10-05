import numpy as np
from time import sleep
import os
import sys

# Add project root to Python path to import the Dobot API
summer_project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, summer_project_dir)
from dobot_function.dobot_api import DobotApi, DobotApiDashboard, DobotApiMove

def connect(ip="192.168.1.6", move_port=30003, feedback_port=30004, dashboard_port=29999):
    # Establishes a connection with the Dobot robot.
    try:
        print("Connecting....")
        move = DobotApiMove(ip, move_port)
        dashboard = DobotApiDashboard(ip, dashboard_port)
        feedback = DobotApi(ip, feedback_port)
        print("connecting successfully.....")
        return move, dashboard, feedback
    except Exception as e:
        print("Connection Failed!")
        return None, None, None

def random_move(move, x_range, y_range, z_range, r_range):
    # Performs a single random move within a predefined safe area.
    x = np.random.uniform(*x_range)
    y = np.random.uniform(*y_range)
    z = np.random.uniform(*z_range)
    r = np.random.uniform(*r_range)

    move.MovJ(x, y, z, r)
    move.Sync()

if __name__ == "__main__":
    # This block is for testing the connection.
    move, dashboard, feedback = connect()

    if move and dashboard:
        dashboard.EnableRobot()
        
        # Define a safe working area for the test move.
        x_range = (240, 300)
        y_range = (-50, 50)
        z_range = (0, 50)
        r_range = (-10, 10)

        print("Performing a random move to test connection...")
        random_move(move, x_range, y_range, z_range, r_range)
        print("Move complete.")
        
        sleep(2)
        
        dashboard.DisableRobot()
        print("Test finished. Robot disabled.")
    else:
        print("Could not run test. Check connection.")
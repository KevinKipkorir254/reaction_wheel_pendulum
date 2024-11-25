import pybullet as p
import sys
import pybullet_data
import time

# Connect to the physics engine
p.connect(p.GUI)

# for m1 mac: disable side window for m1 because it is slow to render.
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

p.resetDebugVisualizerCamera(2, 75, -30, [0,0,0])

# Set gravity
p.setGravity(0, 0, -9.81)

# load the ground
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # plane.urdf is in pybullet_data
p.loadURDF("plane.urdf")

# load the robot
# TODO figure out why some of the pieces of the simulation are not running correctly.

# Load the URDF file (replace 'robot.urdf' with your file)
robot_id = p.loadURDF("robot.urdf")

spinner_chars = ['|', '/', '-', '\\']
i = 0
# Simulate for 100 steps
while True:
    p.stepSimulation()
    sys.stdout.write(f'\r{spinner_chars[i]} Loading...')
    sys.stdout.flush()
    if(i > 2):
        i = 1
    i = i + 1
    time.sleep(1/240)  # Simulate at 240 Hz


# Brian Lesko   1/22/24   Robotics Engineer

import streamlit as st
import pybullet as p
import pybullet_data as pd
import numpy as np
from PIL import Image
import customize_gui # streamlit GUI modifications
gui = customize_gui.gui()
import dualsense 
DualSense = dualsense.DualSense
import robot
my_robot = robot.two2_robot()
import math

def main():
    # Set up the app UI
    gui.clean_format(wide=True)
    gui.about(text = "This code renders and simulates a racecar using pybullet Physics. Use a PS5 remote to control the car")
    Title, subTitle, Sidebar, image_spot = st.empty(), st.empty(), st.sidebar.empty(), st.columns([1,5,1])[1].empty()
    st.subheader("Pybullet Physics Engine")
    
    try: 
        vendorID, productID = int("0x054C", 16), int("0x0CE6", 16)
        ds = DualSense(vendorID, productID)
        ds.connect()
    except Exception as e: st.error("Error occurred while connecting to Dualsense controller. Make sure the controller is wired up and the vendor and product ID's are correctly set in the python script.")    

    # Start PyBullet in DIRECT mode
    try: p.disconnect()
    except: pass
    p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(enableFileCaching=1)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF('plane.urdf')
    p.setPhysicsEngineParameter(numSolverIterations=3)
    p.setTimeStep(0.009)
    
    # Load robot URDF
    xarm = p.loadURDF("racecar/racecar.urdf", flags=p.URDF_INITIALIZE_SAT_FEATURES)
    assert isinstance(xarm, int), "xarm should be an integer"

    # Pybullet Camera Controls 
    width, height = 560, 560
    aspect = width / height
    fov, near, far = 60, .01, 11
    col1, col2, col3 = st.columns([1,5,1])
    with col2: image_placeholder = st.empty()

    # Sim variable initialization
    p.getNumJoints(xarm)
    thetas = [0,0,0,0,0,0,0]
    prev_L1, prev_R1 = False, False
    th = robot.CyclicVariable(thetas)
    angle = 0
    z, x, y = .5, .5, 1
    view_matrix = p.computeViewMatrix(cameraEyePosition=[x, y, z],cameraTargetPosition=[0, 0, .5],cameraUpVector=[0, 0, .5])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    p.setGravity(0, 0, -9.8*1.5)
    f = 1/3
    Steps = 0

    # Set the trigger resistance to 20%
    ds.set_trigger(intensities=[20,20,20,20,20,20,20])
    ds.send_outReport()

    # Simulation Loop
    targetValue = 0.5
    while True: 
        # Receive PS5 Dualsense Controller Data
        ds.receive()
        ds.updateTriggers()
        ds.updateThumbsticks()

        thetas = np.zeros(7)

        # Steering Control 
        # use the left thumbstick to control the steering of the robot
        if abs(ds.LX) > 4:
            thetas[3] = -ds.LX/19
            thetas[5] = -ds.LX/19

        # Throttle Control
        thetas[[1, 2, 4, 6]] = np.where(ds.L2 > 0, -ds.L2 * f, ds.R2 * f)

        # Control the throttle
        jointIndices = [1,2,3,4,5,6,7]  # Assuming thetas now contains target velocities for all joints
        targetSpeeds = [thetas[0],thetas[1], thetas[2], thetas[3], thetas[4], thetas[5], thetas[6]]
        controlModes = p.VELOCITY_CONTROL
        p.setJointMotorControlArray(bodyIndex=1, jointIndices=jointIndices, controlMode=controlModes, targetVelocities=targetSpeeds)

        # Control the steering
        jointIndices = [4,6]  # Joint indices for joints 6 and 4
        targetPositions = [thetas[3], thetas[5]]  # Target speeds for joints 6 and 4
        controlModes = p.POSITION_CONTROL  # Control mode for joints 6 and 4
        p.setJointMotorControlArray(bodyIndex=1, jointIndices=jointIndices, controlMode=controlModes, targetVelocities=targetPositions)

        Steps += 1

        # Capture image with OpenGL renderer
        if Steps % 7 == 0:

            # Get the car's position and orientation
            car_pos, car_orient = p.getBasePositionAndOrientation(xarm)
            car_orient_euler = p.getEulerFromQuaternion(car_orient)

            # Calculate the camera's eye position based on the car's orientation
            x = car_pos[0] + ds.RY/600 + math.cos(car_orient_euler[2]+ math.pi)*.7
            y = car_pos[1] + ds.RX/600 + math.sin(car_orient_euler[2] + math.pi)*.7
            z = car_orient_euler[1]  + car_pos[2]/4 + .4

            # Calculate a point in front of the car based on the car's orientation
            target_x = car_pos[0] + math.cos(car_orient_euler[2]) + ds.RX/200
            target_y = car_pos[1] + math.sin(car_orient_euler[2]) + ds.RY/200
            target_z = car_pos[2]/4 + .3
            target = [target_x, target_y, target_z]

            # Update the camera's target position to follow the car
            view_matrix = p.computeViewMatrix(cameraEyePosition=[x, y, z], cameraTargetPosition=target, cameraUpVector=[0, 0, .6])

            # Capture image with OpenGL renderer
            images = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgba_buffer_opengl = np.resize(np.array(images[2], dtype=np.uint8), (height, width, 4))
            image_placeholder.image(rgba_buffer_opengl[:, :, :3], caption='OpenGL Renderer', use_column_width=True)

        p.stepSimulation()

main()
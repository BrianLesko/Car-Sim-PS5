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
    gui.about(text = "This code renders a simple cube in the physics engine called pybullet. Use a PS5 remote to control the robot.")
    Title, subTitle, Sidebar, image_spot = st.empty(), st.empty(), st.sidebar.empty(), st.columns([1,5,1])[1].empty()
    
    # Setting up the dualsense controller connection
    vendorID, productID = int("0x054C", 16), int("0x0CE6", 16)
    ds = DualSense(vendorID, productID)
    try: ds.connect()
    except Exception as e: st.error("Error occurred while connecting to Dualsense controller. Make sure the controller is wired up and the vendor and product ID's are correctly set in the python script.")    

    # Start PyBullet in DIRECT mode
    try: p.disconnect()
    except: pass
    p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(enableFileCaching=1)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF('plane.urdf')
    p.setPhysicsEngineParameter(numSolverIterations=3)
    
    # Load robot URDF
    xarm = p.loadURDF("xarm/xarm6_robot.urdf", flags=p.URDF_INITIALIZE_SAT_FEATURES, useFixedBase=True)
    # Disable collisions for the robot
    for i in range(p.getNumJoints(xarm)):
        p.setCollisionFilterGroupMask(xarm, i, collisionFilterGroup=0, collisionFilterMask=0)

    # Pybullet Camera Controls 
    width, height = 720, 720
    aspect = width / height
    fov, near, far = 60, .01, 11
    col1, col2, col3 = st.columns([1,5,1])
    with col2: image_placeholder = st.empty()

    # Sim variable initialization
    p.getNumJoints(xarm)
    thetas = [0,0,0,0,0,0]
    prev_L1, prev_R1 = False, False
    th = robot.CyclicVariable(thetas)
    step = .001
    angle = 0
    z, x, y = .5, .5, 1
    view_matrix = p.computeViewMatrix(cameraEyePosition=[x, y, z],cameraTargetPosition=[0, 0, .5],cameraUpVector=[0, 0, .5])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    controlMode = p.POSITION_CONTROL

    # Simulation Loop
    targetValue = 0.5
    while True: 
        # Receive PS5 Dualsense Controller Data
        ds.receive()
        ds.updateTriggers()
        ds.updateThumbsticks()

        # Determine which joint is selected
        joints = [f"<span style='font-size:{'30' if i == th.index else '20'}px;'>J{i+1}</span>" for i in range(6)]
        with Title: 
            st.markdown(f" &nbsp; &nbsp; &nbsp; &nbsp;<L1/R1> &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;{' &nbsp; | &nbsp; '.join(joints)} &nbsp; ", unsafe_allow_html=True)

        # Increment and decrement the controlled joint index based on the L1 and R1 buttons
        if ds.L1 and not prev_L1:
            th.decrement()
        if ds.R1 and not prev_R1:
            th.increment()
        prev_L1 = ds.L1
        prev_R1 = ds.R1

        # Button Control
        i = th.index
        thetas[i] = 0 
        if ds.L2 > 0:
            thetas[i] = - ds.L2/10 #thetas[i] + step*ds.L2/6
        if ds.R2 > 0:
            thetas[i] = ds.R2/10 #thetas[i] - step*ds.R2/6
        
        p.stepSimulation()

        # Control the robot
        jointIndex = i+1
        targetSpeed = thetas[i]  # Assuming thetas now contains target velocities
        p.setJointMotorControl2(bodyUniqueId=xarm, jointIndex=jointIndex, controlMode=p.VELOCITY_CONTROL, targetVelocity=targetSpeed)

        # Camera control 
        if abs(ds.RX) > 4 or abs(ds.RY) > 4:
            if abs(ds.RX) > 4:
                # Update angle based on thumbstick input and calculate new camera position
                angle += ds.RX/700
                x = math.cos(angle)
                y = math.sin(angle)

            # Control camera tilt using ds.RY
            if abs(ds.RY) > 4:
                z += ds.RY/700  # Adjust the divisor as needed
                z = max(.5, min(1.5, z))

            # Camera setup
            view_matrix = p.computeViewMatrix(cameraEyePosition=[x, y, z],cameraTargetPosition=[0, 0, .5],cameraUpVector=[0, 0, .5])
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

        # Capture image with OpenGL renderer
        images = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgba_buffer_opengl = np.resize(np.array(images[2], dtype=np.uint8), (height, width, 4))
        image_placeholder.image(rgba_buffer_opengl[:, :, :3], caption='OpenGL Renderer', use_column_width=True)

main()
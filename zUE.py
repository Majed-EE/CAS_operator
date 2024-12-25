
# Flask server
from flask import Flask, request, jsonify
import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import matplotlib.pyplot as plt
import threading 

global angle_desired
angle_desired=None
app = Flask(__name__)

@app.route('/receive', methods=['POST'])
def receive_data():
    global angle_desired
    data = request.json  # Receive JSON data
    print(f"Flask Received data: {data}")
    angle_desired=data['user_input']
    return {"message": "Data received successfully"}, 200

import logging

# Create a custom logger
logger = logging.getLogger("MyLogger")
logger.setLevel(logging.DEBUG)  # Set the logging level

# Create handlers
console_handler = logging.StreamHandler()  # Console handler
console_handler.setLevel(logging.INFO)  # Log INFO and above to console

file_handler = logging.FileHandler("logfile.log")  # File handler
file_handler.setLevel(logging.DEBUG)  # Log DEBUG and above to file

# Create formatters and add them to handlers
console_format = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

console_handler.setFormatter(console_format)
file_handler.setFormatter(file_format)

# Add handlers to the logger
logger.addHandler(console_handler)
logger.addHandler(file_handler)



logger.debug('Start of program')





finger_1=[2,3,4] # Ring
finger_2=[7,8,9] # Middle
finger_3=[12,13,14] # Index
finger_4=[16,18,19] # Thumb
target_position=[math.radians(80)]



# Finger information
finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
finger_dict={"Ring":{"Joints":finger_1,"Index":5},
            "Index":{"Joints":finger_3,"Index":15},
             "Middle":{"Joints":finger_2,"Index":10},
             "Pinky":{"Joints":None,"Index":None} }





physicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8) # x,y,z direction


############################## Camera position ###############################
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(cameraDistance=0.4,cameraYaw=50,cameraPitch=-50,cameraTargetPosition=[0,0,0])
############################### Camera ###########################

# define start pose
StartPos_cuboid=[0.1,0,0.03998089919150022] 
StartPos_sphere=[-1,0,0.049987949685306586]  
StartPos_arh=[0,0,0]
StartOrientation=p.getQuaternionFromEuler([0,0,0])


planeId=p.loadURDF("plane.urdf")
cuboidId=p.loadURDF(r"models_pybull/simple_object/box.urdf",StartPos_cuboid)
sphereId=p.loadURDF(r"models_pybull/simple_object/sphere.urdf",StartPos_sphere)
arhId=p.loadURDF(r"models_pybull/end_effector/left.urdf",StartPos_arh)
# armId=p.loadURDF(r"models/end_effector/arm.urdf",StartPos_sphere)
# ur5Id=p.loadURDF(r"models/end_effector/left.urdf")



####### plot graph









##### draw axis lines ###########

# Function to draw axes at a given position and orientation
def draw_axes(body_id, link_id=-1, length=0.1, duration=0):
    # Get the position and orientation of the body or link
    if link_id == -1:
        position, orientation = p.getBasePositionAndOrientation(body_id)
    else:
        position, orientation = p.getLinkState(body_id, link_id)[:2]

    # Convert quaternion to rotation matrix
    rotation_matrix = p.getMatrixFromQuaternion(orientation)

    # Define the axes directions
    x_axis = [length * rotation_matrix[0], length * rotation_matrix[3], length * rotation_matrix[6]]
    y_axis = [length * rotation_matrix[1], length * rotation_matrix[4], length * rotation_matrix[7]]
    z_axis = [length * rotation_matrix[2], length * rotation_matrix[5], length * rotation_matrix[8]]

    # Draw the axes
    p.addUserDebugLine(position, [position[0] + x_axis[0], position[1] + x_axis[1], position[2] + x_axis[2]], [1, 0, 0], 2, duration)  # X-axis in red
    p.addUserDebugLine(position, [position[0] + y_axis[0], position[1] + y_axis[1], position[2] + y_axis[2]], [0, 1, 0], 2, duration)  # Y-axis in green
    p.addUserDebugLine(position, [position[0] + z_axis[0], position[1] + z_axis[1], position[2] + z_axis[2]], [0, 0, 1], 2, duration)  # Z-axis in blue

# Draw the axes for the box
# draw_axes(arhId)



# Function to draw the origin axes
def draw_origin_axes(length=0.6, duration=0):
    origin = [0, 0, 0]
    
    # X-axis in red
    p.addUserDebugLine(origin, [length, 0, 0], [1, 0, 0], 2, duration)
    # Y-axis in green
    p.addUserDebugLine(origin, [0, length, 0], [0, 1, 0], 2, duration)
    # Z-axis in blue
    p.addUserDebugLine(origin, [0, 0, length], [0, 0, 1], 2, duration)

# Draw the origin axes
draw_origin_axes()








# set constraints
p.createConstraint(parentBodyUniqueId=arhId,
                   parentLinkIndex=-1,
                   childBodyUniqueId=-1,
                   childLinkIndex=0,
                   jointType=p.JOINT_FIXED,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, 0],
                   childFramePosition=[0, 0, 0],
                   parentFrameOrientation=[0, 0, 0, 1],
                   childFrameOrientation=[0, 0, 0, 1])




# ur5Id=p.loadURDF(r"left.urdf")
jointsnum=p.getNumJoints(sphereId)
# print(jointsnum)
# AABB returns a tuple of two points (min and max corners of the AABB)

aabb_cuboid = p.getAABB(cuboidId)
aabb_sphere = p.getAABB(sphereId)
# aabb_min = aabb[0]
# aabb_max = aabb[1]
threshold = 1e-5
# Calculate the dimensions
dimension_cuboid = [aabb_cuboid[1][i] - aabb_cuboid[0][i] for i in range(3)]
print(f"initial dimension cuboid {dimension_cuboid}")



#### arh control
tips_finger=[5,10,15,20]
for tip in tips_finger:
    p.enableJointForceTorqueSensor(
    bodyUniqueId=arhId,
    jointIndex=tip,
    enableSensor=1
    )






############# simulation ##########################
fx=[]
fy=[]
fz=[]
flag_touch=False
ch=0
ch_step=0






logger.debug("Main program started")








def controller_output(bend_angle):
    # bend_anlge=float(input("enter bend angle:"))
    # finger_id=1
    Force_applied = 100
    finger=finger_1
       #  [11]#finger_3 is index

    logger.debug(f"start of loop controller finge id ={finger}")


    target_bend = bend_angle
    tarPos=[target_bend]*len(finger)
    logger.debug(f"target Position  {tarPos}")
    logger.debug(f"target bend= {target_bend}")
    # lock.acquire()
    p.setJointMotorControlArray(
    bodyUniqueId=arhId,
    jointIndices=finger,
    controlMode=p.POSITION_CONTROL,
    targetPositions=[target_bend]*len(finger),
    forces=[Force_applied]*len(finger))
    # lock.release()
    # logger.debug(f"controller loop{x}")
#     # time.sleep(3)









def sim_loop(lock):
    logger.debug("start of sim loop")
    global angle_desired
    for c_step in range(100):
        p.stepSimulation()
        time.sleep(1./10.)
        #### Sphere spatial information
        position, orientation = p.getBasePositionAndOrientation(sphereId)
        filtered_position_sphere = list(dim if abs(dim) > threshold else 0 for dim in position)
        # print(f"Filtered position of sphere (x, y, z): {filtered_position_sphere}")
        # for tip in tips_finger:
        #     joint_state=p.getJointState(
        #         bodyUniqueId=arhId, 
        #         jointIndex=tip)
        #     print(f"joint info of joint {tip}")
        #     print(joint_state)
                # for tip in tips_finger:
        tip=15
        joint_state=p.getJointState(
            bodyUniqueId=arhId, 
            jointIndex=tip)
        logger.debug(f"joint info of joint {tip} joint state {joint_state[2][:3]}")
        


        fx.append(joint_state[2][0])
        fy.append(joint_state[2][1])
        fz.append(joint_state[2][2])
        # joint_state[2][:3]("contact point info")
        contact_info=p.getContactPoints(arhId,cuboidId,15)
        if angle_desired==None:
            bend_angle=0
            controller_output(bend_angle)
        else:
            bend_angle=int(angle_desired)
            controller_output(bend_angle)
        logger.info(f"bend angle is {angle_desired}")
        # print(len(contact_info))
        if len(contact_info)>0:
            
            
            if len(contact_info)-ch>0:
                ch=len(contact_info)
                print("change step")
                ch_step=c_step
                print(c_step)


        #### cuboid spatial information
        position, orientation = p.getBasePositionAndOrientation(cuboidId)
        filtered_position_cuboid = list(dim if dim > threshold else 0 for dim in position)
        # print(f"Filtered position of cuboid (x, y, z): {filtered_position_cuboid}")
        # print(position,orientation)
        dimension_cuboid = [aabb_cuboid[1][i] - aabb_cuboid[0][i] for i in range(3)]

    print("############## End of LOOP ##########################")

lock = threading.Lock()

# thread1_control_loop = threading.Thread(target=controller_output,args=(lock,))
thread2_sim_loop = threading.Thread(target=sim_loop,args=(lock,))

# thread1_control_loop.start()
thread2_sim_loop.start()

# 
# thread2_sim_loop.join()

if __name__ == '__main__':
    app.run(port=5000)  # Server runs on port 5000
    thread2_sim_loop.join()
    logger.debug("end of main loop")
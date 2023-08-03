### importing

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import sys
import time
from IKR import IKR
import logging
import csv
from roboticstoolbox import trapezoidal
from spatialmath.base import *



# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

###### Determine the appropriate getch() function based on the platform
if sys.platform.startswith('win'):
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import tty
    import termios

    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            char = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return char


######### initialzing the remote coneecteion
logger.info('Program started')
client = RemoteAPIClient()
sim = client.getObject('sim')

rconf = np.array([1,-1,1])
# shoulder = +1 means turned shoulder,      -1 means unturned shoulder
# elbow    = +1 means elbow on the below,   -1 means elbow on the top
# wrist    = +1 means unturned wrist,       -1 means turned wrist

#### Parameteres
nj = 7

############ Real_time control parameteres
time_duration = 1
control_frequency = 50
num_points = int(time_duration * control_frequency)
time_range = np.linspace(0,time_duration,num_points)

######### setting target joint pos func
def setJointPosition(joint_number ,joint_position):
    sim.setJointTargetPosition(joint_handle[joint_number], joint_position)

######## defining object handles 
joint_handle = []
joint_handle.append(sim.getObject('/LBRiiwa14R820/joint'))
for i in range(1,7):
    joint_handle.append(sim.getObject(f'/LBRiiwa14R820/link{i+1}_resp/joint'))
world_frame_handle = sim.getObject('/DefaultLights')
tool_handle = sim.getObject('/LBRiiwa14R820/link7_resp/joint')

###### Program 
sim.startSimulation()

###### Home Configuration 
###### option 1: joints = np.deg2rad([0,30,0,-50,0,0,0]) >>  pose [ 0.679  0   0.806]
###### option 2: joints = np.deg2rad([0,0,0,-90,0,90,0]) >>  pose [ 0.400  0   0.704]
###### option 3: joints = np.deg2rad([0,30,0,-80,0,80,0]) >>  pose [ 0.400  0   0.704]
# # 1
# setJointPosition(1,np.deg2rad(30))
# time.sleep(0.5)
# setJointPosition(3,np.deg2rad(-50))
# time.sleep(0.5)
# 2
setJointPosition(3,np.deg2rad(-90))
time.sleep(3)
setJointPosition(5,np.deg2rad(90))
time.sleep(3)
# # # 3
# setJointPosition(1,np.deg2rad(30))
# time.sleep(3)
# setJointPosition(3,np.deg2rad(-80))
# time.sleep(3)
# setJointPosition(5,np.deg2rad(80))
# time.sleep(3)


######### Getting pos and orn of the end effector
tool_position = sim.getObjectPosition(tool_handle,world_frame_handle)
tool_orientation = sim.getObjectQuaternion(tool_handle,world_frame_handle)
pos_x = tool_position[0]
pos_y = tool_position[1]
pos_z = tool_position[2]
logger.info(f'Current Tool X Position: {pos_x:.{3}f}')
logger.info(f'Current Tool Y Position: {pos_y:.{3}f}')
logger.info(f'Current Tool Z Position: {pos_z:.{3}f}')

# reading joint angles
#joint_angle = np.array([])
joint_angle = np.zeros(nj)
for i in range(nj):
    angle = sim.getJointPosition(joint_handle[i])
    #joint_angle = np.append(joint_angle,angle)
    joint_angle[i] = angle
    #logger.info(f'Joint number {i+1} position is {angle:.{2}f}')
#print(tool_orientation)

#### desired pose

## 1
# pose = np.array([[0,0,1,0.5],
#               [0,1,0,0.3],
#               [-1,0,0,0.8],
#               [0,0,0,1]])

# 2
pose = T = transl(0.5,0.3,0.1) @ troty(180, 'deg')

##### Generating a set of joint angle for final pose
j_final = IKR(pose,rconf,joint_angle)

#### Generating a desired trajectory for the each joint
joint_traj = np.zeros(nj, dtype=object)
for i in range(nj):
    #logger.info(f'joint_angle[{i}] is {joint_angle[i]:.{2}f} and j_final[0,{i}] is {j_final[0,i]:.{2}f}')
    joint_traj[i] = trapezoidal(joint_angle[i],j_final[0,i],time_duration * control_frequency)

# Reaching to target pose
dja = np.rad2deg(joint_angle)
results = [[0,tool_position[0],tool_position[1],tool_position[2],dja[0],dja[1],dja[2],dja[3],dja[4],dja[5],dja[6]]]
n = time_duration * control_frequency
for t in range(n):
    for i in range(0,7):
        joint_value = joint_traj[i].q[t]
        sim.setJointTargetPosition(joint_handle[i],joint_value)

    time.sleep(1/control_frequency)

    for i in range(7):
        angle = sim.getJointPosition(joint_handle[i])
        #joint_angle = np.append(joint_angle,angle)
        joint_angle[i] = angle
    tool_position = sim.getObjectPosition(tool_handle,world_frame_handle)
    dja = np.rad2deg(joint_angle)
    results.append([t+1,tool_position[0],tool_position[1],tool_position[2],dja[0],dja[1],dja[2],dja[3],dja[4],dja[5],dja[6]])


for i in range(7):
    joint_angle = np.rad2deg(sim.getJointPosition(joint_handle[i]))
    logger.info(f'Joint number {i+1} position is {joint_angle:.{2}f}')

logger.info('Reached to desired angles.')
tool_position = sim.getObjectPosition(tool_handle,world_frame_handle)
#tool_orientation = sim.getObjectQuaternion(tool_handle,world_frame_handle)
pos_x = tool_position[0]
pos_y = tool_position[1]
pos_z = tool_position[2]
logger.info(f'Current Tool X Position: {pos_x:.{3}f}')
logger.info(f'Current Tool Y Position: {pos_y:.{3}f}')
logger.info(f'Current Tool Z Position: {pos_z:.{3}f}')
#logger.info(tool_orientation)

# Save results to a CSV file
output_file = "Joint_results.csv"
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["step", "x", "y", "z", "J1", "J2", "J3", "J4" , "J5" , "J6" , "J7"])
    writer.writerows(results)

##### closing the simulation
logger.info('Press q to quit:')
while True:
    user_input = getch()
    if user_input.lower() == 'q':
        sim.stopSimulation()  # Call the stopSimulation() function
        break  # Exit the loop

# finishing the program
logger.info('Program ended')
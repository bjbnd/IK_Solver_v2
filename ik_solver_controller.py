from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import sys
import time
from IK.InverseKinematics import InverseKinematics


# Determine the appropriate getch() function based on the platform
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


# initialzing the remote coneecteion
print('Program started')
client = RemoteAPIClient()
sim = client.getObject('sim')

# hyper pararmeteres 
joint_velocity = np.deg2rad(30)
joint_limit = [170,120,170,120,170,120,175]
## Real_time control parameteres
time_duration = 5
control_frequency = 50
num_points = int(time_duration * control_frequency)
time_range = np.linspace(0,time_duration,num_points)

# setting target joint pos func
def setJointPosition(joint_number ,joint_position):
    sim.SetJointTargetVelocity(joint_handle[joint_number], joint_velocity)
    sim.setJointTargetPosition(joint_handle[joint_number],np.deg2rad(joint_position))
# cubic trajectory
def cubic_trajectory(t,q0,qf):
    t = t/5
    joint_value = q0 + 3*(qf-q0)*t**2 - 2*(qf-q0)*t**3
    return joint_value
# defining object handles 
joint_handle = []
joint_handle.append(sim.getObject('/LBRiiwa14R820/joint'))
for i in range(1,7):
    joint_handle.append(sim.getObject(f'/LBRiiwa14R820/link{i+1}_resp/joint'))
world_frame_handle = sim.getObject('/DefaultLights')
tool_handle = sim.getObject('/LBRiiwa14R820/link7_resp/joint')

# Program 
sim.startSimulation()
tool_position = sim.getObjectPosition(tool_handle,world_frame_handle)
tool_orientation = sim.getObjectQuaternion(tool_handle,world_frame_handle)
pos_x = tool_position[0]
pos_y = tool_position[1]
pos_z = tool_position[2]
print(f'Current Tool X Position: {pos_x:.{3}f}')
print(f'Current Tool Y Position: {pos_y:.{3}f}')
print(f'Current Tool Z Position: {pos_z:.{3}f}')
#print(tool_orientation)
# desired trajectory  and configuration 
T = np.array([[0,0,1,0.5],
              [0,1,0,0.3],
              [-1,0,0,0.8],
              [0,0,0,1]])

psi = np.deg2rad(60)

rconf = np.array([-1,-1,-1])
[j,_,_] = InverseKinematics(T,psi,rconf)
desired_angle = np.rad2deg(j)

#     print(f'Joint number {i+1} is rotating in positive direction')
#     limit = joint_limit[i]
#     for t in time_range:
#         # setting the joint angles
#         joint_value = np.deg2rad(cubic_trajectory(t,0,limit))
#         sim.setJointTargetPosition(joint_handle[i],joint_value)
#         time.sleep(1/control_frequency)
#     print(f'Joint number {i+1} is rotating in negative direction')
#     for t in time_range:
#         # setting the joint angles
#         joint_value = np.deg2rad(cubic_trajectory(t,0,-limit))
#         sim.setJointTargetPosition(joint_handle[i],joint_value)
#         time.sleep(1/control_frequency)
#     sim.setJointTargetPosition(joint_handle[i],0)

for t in time_range:
    # setting the joint angles
    for i in range(0,7):
        joint_value = np.deg2rad(cubic_trajectory(t,0,desired_angle[0,i]))
        sim.setJointTargetPosition(joint_handle[i],joint_value)
    time.sleep(1/control_frequency)
for i in range(7):
    joint_angle = np.rad2deg(sim.getJointPosition(joint_handle[i]))
    print(f'Joint number {i+1} position is {joint_angle:.{2}f}')

print('Reached to desired angles.')
tool_position = sim.getObjectPosition(tool_handle,world_frame_handle)
#tool_orientation = sim.getObjectQuaternion(tool_handle,world_frame_handle)
pos_x = tool_position[0]
pos_y = tool_position[1]
pos_z = tool_position[2]
print(f'Current Tool X Position: {pos_x:.{3}f}')
print(f'Current Tool Y Position: {pos_y:.{3}f}')
print(f'Current Tool Z Position: {pos_z:.{3}f}')
#print(tool_orientation)
##### closing the simulation
print('Press q to quit:')
while True:
    user_input = getch()
    if user_input.lower() == 'q':
        sim.stopSimulation()  # Call the stopSimulation() function
        break  # Exit the loop

# finishing the program
print('Program ended')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import logging

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

######### initialzing the remote coneecteion
logger.info('Program started')
client = RemoteAPIClient()
sim = client.getObject('sim')

######## defining object handles 
joint_handle = []
joint_handle.append(sim.getObject('/LBRiiwa14R820/joint'))
for i in range(1,7):
    joint_handle.append(sim.getObject(f'/LBRiiwa14R820/link{i+1}_resp/joint'))
world_frame_handle = sim.getObject('/DefaultLights')
tool_handle = sim.getObject('/LBRiiwa14R820/link7_resp/joint')

###### Program 
sim.startSimulation()


######### Getting linear velocity of the end effector
tool_velocity = sim.getObjectVelocity(tool_handle)[0]

for i in range(len(tool_velocity)):
    print(tool_velocity[i])

for i in range(len(joint_handle)):
    tmp_vel = sim.getJointTargetVelocity(joint_handle[i])
    print(tmp_vel)

# finishing the program
sim.stopSimulation()
logger.info('Program ended')
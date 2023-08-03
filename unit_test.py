import numpy as np

from IK.ForwardKinematics import ForwardKinematics

joints = np.deg2rad([0,30,0,-50,0,0,0])
pose,_,_,_ = ForwardKinematics(joints)

p = pose[:3, 3]
x = p[0]
y = p[1]
z = p[2]

print("Position X: {:.3f}".format(x))

print("Position Y: {:.3f}".format(y))

print("Position Z: {:.3f}".format(z))
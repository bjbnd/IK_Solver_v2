import numpy as np
import math
import warnings
#from skew import skew
#from unit import unit
#from .dh_calc import dh_calc
from Utils.dh_calc import dh_calc
from .ReferencePlane import ReferencePlane
from Utils.skew import skew
from Utils.vector_matrix import vector_matrix
import logging

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

def InverseKinematics(pose, nsparam, rconf):
    # INVERSEKINEMATICS
    #
    # Calculates the inverse kinematics for a KUKA LBR iiwa manipulator.
    #
    # Input:  pose    - Homogeneous Matrix size(4,4)
    #         nsparam - Arm Angle
    #         rconf   - Robot Configuration 3D vector
    # Output: joints  - Joint values size(1,7).
    #         s_mat   - Shoulder joint matrices As, Bs and Cs
    #         w_mat   - Wrist joint matrices Aw, Bw and Cw

    arm, elbow, wrist = rconf[0],rconf[1],rconf[2]
    # Tolerance
    tol = 1e-8

    # Robot parameteres
    # Link length DO NO FORGET TO CHANGE THE LINKS IN ReferencePlane.py file
    l = [0.36, 0.42, 0.4, 0.0761]

    # Denavit-Hartenberg parameters 7 DoF
    # DH: [a, alpha,    d, theta] 
    dh = np.array([[0, -np.pi/2, l[0], 0],
                   [0, np.pi/2, 0, 0],
                   [0, np.pi/2, l[1], 0],
                   [0, -np.pi/2, 0, 0],
                   [0, -np.pi/2, l[2], 0],
                   [0, np.pi/2, 0, 0],
                   [0, 0, l[3], 0]])
    # number of joints
    nj = dh.shape[0]
    # joints value of virtual manipulator
    joints = np.zeros((1, nj))
    # Shoulder rotation matrices
    s_mat = np.zeros((3, 3, 3))
    # Wrist rotation matrices
    w_mat = np.zeros((3, 3, 3))

    xend = pose[:3, 3] # end-effector position from base
    xs = np.array([0, 0, dh[0, 2]]) # shoulder position from base
    xwt = np.array([0, 0, dh[-1, 2]]) # end-effector position from base
    xw = xend - np.dot(pose[:3, :3], xwt) #wrist position from base
    xsw = xw - xs #shoulder to wrist vector
    usw = xsw / np.linalg.norm(xsw)

    lbs = l[0]
    lse = l[1]  # upper arm length (shoulder to elbow)
    lew = l[2]  # lower arm length (elbow to wrist)

    # Check if pose is within arm+forearm reach
    assert np.linalg.norm(xsw) < lse + lew and np.linalg.norm(xsw) > lse - lew, 'Specified pose outside reachable workspace'
    # if np.linalg.norm(xsw) < lse + lew and np.linalg.norm(xsw) > lse - lew:
    #     warnings.warn('Specified pose outside reachable workspace')

    # -- Joint 4 --
    # Elbow joint can be directly calculated since it only depends on the 
    # robot configuration and the xsw vector 
    assert abs((np.linalg.norm(xsw)**2 - lse**2 - lew**2) - (2*lse*lew)) > tol, 'Elbow singularity. Tip at reach limit.'
    # if abs((np.linalg.norm(xsw)**2 - lse**2 - lew**2) - (2*lse*lew)) > tol:
    #     warnings.warn('Elbow singularity. Tip at reach limit.')

    # Cosine law - According to our robot, joint 4 rotates backwards
    joints[0,3] = elbow * np.arccos((((np.linalg.norm(xsw))**2)- lse**2 -lew**2) /(2*lse*lew))
    # Added
    T34 = dh_calc(dh[3, 0], dh[3, 1], dh[3, 2], joints[0, 3])
    R34 = T34[:3, :3]

    # Shoulder Joints
    # First compute the reference joint angles when the arm angle is zero.
    _, R03_o, _ = ReferencePlane(pose, elbow)
    
    skew_usw = skew(usw)
    # Following eq. (15), the auxiliary matrixes As Bs and Cs can be calculated 
    # by substituting eq. (6) into (9). 
    # R0psi = I3 + sin(psi)*skew_usw + (1-cos(psi))*skew_usw²    (6)
    # R03 = R0psi * R03_o                                         (9)
    # Substituting (distributive prop.) we get:
    # R03 = R03_o*skew_usw*sin(psi) + R03_o*(-skew_usw²)*cos(psi) + R03_o(I3 + skew_usw²)
    # R03 =      As       *sin(psi) +        Bs         *cos(psi) +          Cs
    As = np.dot(skew_usw, R03_o)
    Bs = -(skew_usw @ skew_usw) @ R03_o
    Cs = vector_matrix(usw) @ R03_o
    psi = nsparam
    R03 = As * np.sin(psi) + Bs * np.cos(psi) + Cs
    
    # T03 transformation matrix (DH parameters)
    # [ cos(j1)*cos(j2)*cos(j3) - sin(j1)*sin(j3), cos(j1)*sin(j2), cos(j3)*sin(j1) + cos(j1)*cos(j2)*sin(j3), 0.4*cos(j1)*sin(j2)]
    # [ cos(j1)*sin(j3) + cos(j2)*cos(j3)*sin(j1), sin(j1)*sin(j2), cos(j2)*sin(j1)*sin(j3) - cos(j1)*cos(j3), 0.4*sin(j1)*sin(j2)]
    # [                          -cos(j3)*sin(j2),         cos(j2),                          -sin(j2)*sin(j3),  0.4*cos(j2) + 0.34]
    # [                                         0,               0,                                         0,                   1]
    joints[0,0] = np.arctan2(arm * R03[1, 1], arm * R03[0, 1])
    joints[0,1] = arm * np.arccos(R03[2, 1])
    joints[0,2] = np.arctan2(-arm * R03[2, 2], -arm * R03[2, 0])

    Aw = np.dot(np.dot(R34.T, As.T), pose[:3, :3])
    Bw = np.dot(np.dot(R34.T, Bs.T), pose[:3, :3])
    Cw = np.dot(np.dot(R34.T, Cs.T), pose[:3, :3])

    R47 = Aw * np.sin(psi) + Bw * np.cos(psi) + Cw
    # T47 transformation matrix (DH parameters)
    # [ cos(j5)*cos(j6)*cos(j7) - sin(j5)*sin(j7), - cos(j7)*sin(j5) - cos(j5)*cos(j6)*sin(j7), cos(j5)*sin(j6), (63*cos(j5)*sin(j6))/500]
    # [ cos(j5)*sin(j7) + cos(j6)*cos(j7)*sin(j5),   cos(j5)*cos(j7) - cos(j6)*sin(j5)*sin(j7), sin(j5)*sin(j6), (63*sin(j5)*sin(j6))/500]
    # [                          -cos(j7)*sin(j6),                             sin(j6)*sin(j7),         cos(j6),   (63*cos(j6))/500 + 2/5]
    # [                                         0,                                           0,               0,                        1]
    joints[0,4] = np.arctan2(wrist * R47[1, 2], wrist * R47[0, 2])
    joints[0,5] = wrist * np.arccos(R47[2, 2])
    joints[0,6] = np.arctan2(wrist * R47[2, 1], -wrist * R47[2, 0])
    # Grouping Shoulder and Wrist matrices that will be used by the joint
    # limit algorithms
    s_mat[:, :, 0] = As
    s_mat[:, :, 1] = Bs
    s_mat[:, :, 2] = Cs
    w_mat[:, :, 0] = Aw
    w_mat[:, :, 1] = Bw
    w_mat[:, :, 2] = Cw

    return np.array([joints,s_mat,w_mat],dtype=object)





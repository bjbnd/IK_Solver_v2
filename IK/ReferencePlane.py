import numpy as np
from Utils.unit import unit
from Utils.dh_calc import dh_calc
import warnings
import logging

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

def ReferencePlane(pose, elbow):
    tol = 1e-6
    
    # Robot parameters
    l = [0.36, 0.42, 0.4, 0.0761]
    dh = np.array([[0, -np.pi/2, l[0], 0],
                   [0, np.pi/2, 0, 0],
                   [0, np.pi/2, l[1], 0],  # theta3 == 0
                   [0, -np.pi/2, 0, 0],
                   [0, -np.pi/2, l[2], 0],
                   [0, np.pi/2, 0, 0],
                   [0, 0, l[3], 0]])
    
    # Joint values of virtual manipulator
    joints = np.zeros(7)
    #logger.info(f'End-effector position is: {pose[:3, 3]}')
    xend = pose[:3, 3]  # end-effector position from base    
    xs0 = np.array([0, 0, dh[0, 2]])  # shoulder position from base 
    xwt = np.array([0, 0, dh[-1, 2]])  # end-effector position from wrist
    xw0 = xend - np.dot(pose[:3, :3], xwt)  # wrist position from base
    xsw = xw0 - xs0  # shoulder to wrist vector

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


    # Cosine law
    joints[3] = elbow * np.arccos((np.linalg.norm(xsw)**2 - lse**2 - lew**2) / (2*lse*lew))

    # Shoulder Joints
    T34 = dh_calc(dh[3, 0], dh[3, 1], dh[3, 2], joints[3])
    R34 = T34[:3, :3]

    # These are the vectors corresponding to our DH parameters
    xse = np.array([0, lse, 0])
    xew = np.array([0, 0, lew])
    # m = member between parentheses. Check equation (14)
    m = xse + np.dot(R34, xew)

    # -- Joint 1 --
    # Since joint3 is locked as 0, the only joint to define the orientation of
    # the xsw vector in the xy-plane is joint 1. Therefore and since we are
    # only interested in the transformation T03 (disregarding joint limits), we
    # choose to simply set joint 1 as the atan of xsw y and x coordinates 
    # (even if it goes beyond the joint limit).

    # Cannot be this because if x and y are 0, then```
    if np.linalg.norm(np.cross(xsw, np.array([0, 0, 1]))) > tol:
        joints[0] = np.arctan2(xsw[1], xsw[0])
    else:
        joints[0] = 0

    # -- Joint 2 --
    # Can be found through geometric relations
    # Let phi be the angle E-S-W, and theta2 the angle (z-axis)-S-E.
    # Then, theta2 = atan2(r, xsw[2]) -/+ phi.
    # phi can be calculated, as a function of theta3:
    #   atan2(lew*np.sin(theta4), lse+lew*np.cos(theta4))
    # z-axis
    #   ^
    #   |  E O------------O W
    #   |   /        .  
    #   |  /      .
    #   | /    .    xsw
    #   |/  .
    # S O___________________ r-axis
    #
    r = np.hypot(xsw[0], xsw[1])
    dsw = np.linalg.norm(xsw)
    phi = np.arccos((lse**2 + dsw**2 - lew**2) / (2 * lse * dsw))

    joints[1] = np.arctan2(r, xsw[2]) + elbow * phi

    # Lower arm transformation
    T01 = dh_calc(dh[0, 0], dh[0, 1], dh[0, 2], joints[0])
    T12 = dh_calc(dh[1, 0], dh[1, 1], dh[1, 2], joints[1])
    T23 = dh_calc(dh[2, 0], dh[2, 1], dh[2, 2], 0)
    T34 = dh_calc(dh[3, 0], dh[3, 1], dh[3, 2], joints[3])
    T04 = np.dot(np.dot(np.dot(T01, T12), T23), T34)

    rot_base_elbow = np.dot(np.dot(T01[:3, :3], T12[:3, :3]), T23[:3, :3])

    # With T03 we can calculate the reference elbow position and with it the
    # vector normal to the reference plane.
    x0e = T04[:3, 3]  # reference elbow position
    v1 = unit(x0e - xs0)  # unit vector from shoulder to elbow
    v2 = unit(xw0 - xs0)  # unit vector from shoulder to wrist

    ref_plan_vector = np.cross(v1, v2)

    return np.array([ref_plan_vector, rot_base_elbow, joints ],dtype=object) 
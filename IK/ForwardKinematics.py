import numpy as np
from .ReferencePlane import ReferencePlane
from Utils.unit import unit
import logging

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

def ForwardKinematics(joints):
    # FORWARDKINEMATICS
    # 
    #  Calculates the forward kinematics for a KUKA LBR iiwa manipulator.
    #    
    #  Input:    joints  - Joint values size(1,7).
    #  Output:   pose    - Homogeneous Matrix size(4,4)
    #            nsparam - Arm Angle
    #            rconf   - Robot Configuration 8-bit number
    
    # Tolerance
    tol = 1e-8
    pi = np.pi
    # Robot parameteres
    # Link length 
    l = np.array([0.36, 0.42, 0.4, 0.0761])
    # Denavit-Hartenberg parameteres 7 DoF
    # DH: [a, alpha, d, theta]
    dh = np.array([[0,-pi/2,l[0],0],
                   [0,+pi/2,0,0],
                   [0,+pi/2,l[1],0],
                   [0,-pi/2,0,0],
                   [0,-pi/2,l[2],0],
                   [0,+pi/2,0,0],
                   [0,0,l[3],0]])

    #Number of Joints
    nj = dh.shape[0]

    # Robot Configuration
    
    shoulder = 1 if joints[1] >= 0 else -1

    elbow = 1 if joints[3] >= 0 else -1

    wrist = 1 if joints[5] >= 0 else -1

    rconf = np.array([shoulder,elbow,wrist])

    

    # Assign joint values to the theta column of the DH parameters
    #print(f'dh[:,3] is {dh[:,3]}')
    #logger.info(f'Joints value to be passed to dh is: {joints}')
    dh[:,3] = joints
    #print(f'dh[:,3] is {dh[:,3]}')
    #Store transformations from the base reference frame to the index joint
    #e.g: tr(:,:,2) is the T02 -> transformation from base to joint 2 (DH table)
    tr = np.zeros((4, 4, nj))
    #Rotation Matrix applied with Denavit-Hartenberg parameters [same as (3)]
    #R = [Xx,Yx,Zx,   --  Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =  sin(theta) * sin(alpha)
    #     Xy,YY,Zy,   --  Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy = -cos(theta) * sin(alpha)
    #     Xz,Yz,Zz];  --  Xz = 0.0,        Yz =  sin(alpha),              Zz =  cos(alpha) 
    #logger.info(f'Joints value to be used for FK is: {dh[:,3]}')
    for i in range(nj):
        a = dh[i, 0]
        alpha = dh[i, 1]
        d = dh[i, 2]
        theta = dh[i, 3]

        v = np.array([a * np.cos(theta), a * np.sin(theta), d])

        Xx = np.cos(theta)
        Yx = -np.sin(theta) * np.cos(alpha)
        Zx = np.sin(theta) * np.sin(alpha)

        Xy = np.sin(theta)
        Yy = np.cos(theta) * np.cos(alpha)
        Zy = -np.cos(theta) * np.sin(alpha)

        Xz = 0.0
        Yz = np.sin(alpha)
        Zz = np.cos(alpha)

        tmp = np.array([[Xx, Yx, Zx, v[0]],
                        [Xy, Yy, Zy, v[1]],
                        [Xz, Yz, Zz, v[2]],
                        [0, 0, 0, 1]])

        if i == 0:
            tr[:, :, 0] = tmp
        else:
            #tr[:, :, i] = np.dot(tr[:, :, i - 1], tmp)
            tr[:, :, i] = tr[:, :, i - 1] @ tmp
    
    xs = tr[:3, 3, 0]  # shoulder position from base
    xe = tr[:3, 3, 3]  # elbow position from base
    xw = tr[:3, 3, 5]  # wrist position from base
    xsw = xw - xs      # wrist position from shoulder

    pose = tr[:, :, -1]  # end-effector transformation from base
    #logger.info(f'pose afre FK calcualtion is {pose[:3, 3]}')
    # Calculate the nsparam - Arm Angle
    # The arm plane is defined by the plane formed by the xs, xe, and xw points
    # The reference plane is defined by the xs, xw, and xe0 (explained below)
    # . A virtual robotic manipulator is created from the KUKA LBR iiwa manipulator
    #   structure. Everything is the same except the 3rd joint, which is fixed as 0.
    # . Now we compute the Inverse Kinematics to place the virtual robot in the
    #   same pose as the real robot.
    # . The elbow position of this virtual robot is xe0. Thus, with xs, xw, and
    #   xe0 we form the reference plane
    vv, _, jout = ReferencePlane(pose, elbow)

    # vv is the vector normal to the reference plane: xs - xe0 - xw
    # vc is the vector normal to the current plane:   xs - xe - xw
    v1 = unit(xe - xs)
    v2 = unit(xw - xs)
    vc = np.cross(v1, v2)

    cos_ns = np.dot(unit(vv), unit(vc))
    if abs(np.linalg.norm(cos_ns)) > 1:
        cos_ns = np.sign(cos_ns)
        
    # This vector will give the sign of the nsparam
    v3 = np.cross(unit(vv), unit(vc))
    tol = 1e-6  # Adjust the tolerance value as needed
    if np.linalg.norm(v3) > tol:
        nsparam = np.sign(np.dot(v3, xsw)) * np.arccos(cos_ns)
    else:
        if np.linalg.norm(vv - vc) < tol:
            nsparam = 0
        else:
            nsparam = np.pi

    return np.array([ pose, nsparam, rconf, jout ],dtype=object)
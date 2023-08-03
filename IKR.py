import numpy as np
import warnings
import logging
from IK.ForwardKinematics import ForwardKinematics
from Utils.dh_calc import dh_calc
from IK.ReferencePlane import ReferencePlane
from Utils.skew import skew
from Utils.vector_matrix import vector_matrix
from LimitAnalysis.PsiLimits import PsiLimits
from IK.InverseKinematics import InverseKinematics

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

# psi calcualtor function
def expo_psi(psi, sup, inf, k, a2):
    # Calculate the intermediate variables
    delta_psi = (psi - inf) / (sup - inf)
    delta_sup_psi = (sup - psi) / (sup - inf)
    
    # Calculate the result using the given formula
    result = (k * (sup - inf) / 2) * (np.exp(-a2 * delta_psi) - np.exp(-a2 * delta_sup_psi))
    
    return result


def IKR(pose,rconf,current_joint):

    shoulder,elbow,wrist = rconf[0],rconf[1],rconf[2]
    #logger.info(f'Current Joint is :{current_joint}')
    _, psi_current,_,_= ForwardKinematics(current_joint)
    # joint limits 
    jl = np.deg2rad(np.array([170 ,120 ,170 ,120 ,170 ,120 ,170]))
    # next psi calculating parameters
    k = 0.1
    alpha = 5
    # Tolerance
    tol = 1e-8
    # Robot parameteres
    # Link length DO NO FORGET TO CHANGE THE LINKS IN ReferencePlane.py Inversekienamtics.py ForwardKinematics.py
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
    Aw = np.dot(np.dot(R34.T, As.T), pose[:3, :3])
    Bw = np.dot(np.dot(R34.T, Bs.T), pose[:3, :3])
    Cw = np.dot(np.dot(R34.T, Cs.T), pose[:3, :3])
    s_mat[:, :, 0] = As
    s_mat[:, :, 1] = Bs
    s_mat[:, :, 2] = Cs
    w_mat[:, :, 0] = Aw
    w_mat[:, :, 1] = Bw
    w_mat[:, :, 2] = Cw

    psi_allow = PsiLimits(rconf,s_mat,w_mat,jl)
    n = len(psi_allow)
    for i in range(0,n,2):
        lower_limit = psi_allow[i]
        upper_limit = psi_allow[i+1]
        if (psi_current >= lower_limit) & (psi_current <= upper_limit):
            # option 1
            next_psi = expo_psi(psi_current,upper_limit,lower_limit,k,alpha)
            # option 2
            #next_psi = (upper_limit - lower_limit)/2

    #print(f'next chosen psi is: {np.rad2deg(next_psi)}')
    [next_joint,_,_] = InverseKinematics(pose,next_psi,rconf)

    return next_joint 
import numpy as np
import logging
from .TanJoint import TanJoint
from .CosJoint import CosJoint
from Utils.IntersectIntervals import IntersectIntervals
# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)

def PsiLimits(rconf, s_mat, w_mat, jl):
    
    s, e, w = rconf[0],rconf[1],rconf[2]
    
    As = s_mat[:,:,0]
    Bs = s_mat[:,:,1]
    Cs = s_mat[:,:,2]
    
    Aw = w_mat[:,:,0]
    Bw = w_mat[:,:,1]
    Cw = w_mat[:,:,2]
    
    # print(As)
    # print(Bs)
    # print(Cs)

    # print(Aw)
    # print(Bw)
    # print(Cw)
 
    #### Joint 1
    lim1 = TanJoint(s*As[1,1], s*As[0,1], s*Bs[1,1], s*Bs[0,1], s*Cs[1,1], s*Cs[0,1], jl[0])
    logger.info(f'lim 1 is: {np.rad2deg(lim1)}')
    
    #### Joint 2
    lim2 = CosJoint(As[2,1], Bs[2,1], Cs[2,1], s, jl[1])
    logger.info(f'lim 2 is: {np.rad2deg(lim2)}')
    
    #### Joint 3
    lim3 = TanJoint(s*(-As[2,2]), s*(-As[2,0]), s*(-Bs[2,2]), s*(-Bs[2,0]), s*(-Cs[2,2]), s*(-Cs[2,0]), jl[2])
    logger.info(f'lim 3 is: {np.rad2deg(lim3)}')
    
    #### Joint 5
    lim5 = TanJoint(w*Aw[1,2], w*Aw[0,2], w*Bw[1,2], w*Bw[0,2], w*Cw[1,2], w*Cw[0,2], jl[4])
    logger.info(f'lim 5 is: {np.rad2deg(lim5)}')
    
    #### Joint 6
    lim6 = CosJoint(Aw[2,2], Bw[2,2], Cw[2,2], w, jl[5])
    logger.info(f'lim 6 is: {np.rad2deg(lim6)}')
    
    #### Joint 7
    lim7 = TanJoint(w*Aw[2,1], w*(-Aw[2,0]), w*Bw[2,1], w*(-Bw[2,0]), w*Cw[2,1], w*(-Cw[2,0]), jl[6])
    logger.info(f'lim 7 is: {np.rad2deg(lim7)}')
    
    # Here we receive the limits for each of the shoulder and wrist joints.
    lim12 = IntersectIntervals(lim1, lim2)
    lim35 = IntersectIntervals(lim3, lim5)
    lim67 = IntersectIntervals(lim6, lim7)
    lim1235 = IntersectIntervals(lim12, lim35)
    allow_interval = IntersectIntervals(lim1235, lim67)
    logger.info(f'Allowed interval for Psi(degree) is {np.rad2deg(allow_interval)} ')
    return allow_interval

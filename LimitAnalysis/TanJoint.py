import numpy as np
from .ArmAngleFunction import anglemod , ArmAngleFunction
from Utils.IntersectIntervals import IntersectIntervals
import logging

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)


def TanJoint(an,ad,bn,bd,cn,cd,jl):

    # compute the stationary points of the arm angle for a tan joint

    # treshold to detect singularities
    thr = 1e-6

    # The differentiation checks out
    at = bd*cn - bn*cd
    bt = an*cd - ad*cn
    ct = an*bd - ad*bn

    # CTan = at^2 + bt^2 - ct^2;
    # KL1 = 2*atan(at - sqrt(at^2 + bt^2 - ct^2)/(bt - ct));
    # KL2 = 2*atan(at + sqrt(at^2 + bt^2 - ct^2)/(bt - ct));

    # WHAT ABOUT SINGULARITIES???
    if (np.abs(at**2 + bt**2 - ct**2) - thr):
        #print('In TanJoint, case 1 happend')
        # Condition (31): one stationary point exists
        # Both the numerator as the denominator are zero, meaning the arm angle
        # is at a singular point because, the angle theta_i is indeterminate at
        # this arm angle
        sing = anglemod(2 * np.arctan2(at,(bt-ct)))
        safe_distance = np.deg2rad(7)
        sing_lim = np.array([-np.pi,sing - safe_distance,sing + safe_distance,np.pi])
    # Stationary points not always map the theta max or min. Specially when we
    # have configurations with negative shoulder, elbow or wrist values, we
    # tend to have discontinuities in the function theta(psi) eq.23.
    # These discontinuities are not contemplated in Shimizu paper, due to the
    # values reported and the joint limits of his setup.
    # These discontinuities are not singularities, but are related to the 
    # domain of tan. Thus, when a monotonic function reaches -pi or pi a
    # discontinuity happens, and the theta value shifts 2*pi or -2*pi.
    # Since it crosses the joint limit values, they create a new interval where
    # psi values lead to joint limit violation.
    # 1) Check if any psi values cross the joint limits
    pt1 = ArmAngleFunction([an, ad], [bn, bd], [cn, cd], -jl, 1)
    pt2 = ArmAngleFunction([an, ad], [bn, bd], [cn, cd], jl, 1)
    #print(f'pt1 is {pt1} and pt2 is {pt2}')
    ptlim = np.array([pt1,pt2])  # Psi values that cross the limits
    #print(f'ptlim for TanJoint is: {ptlim} and len is {len(ptlim)}')
    # There is at least a psi value that matches a joint limit
    if len(pt1) > 0 or len(pt2) > 0:
        #print('In TanJoint, case 2 happend')
        # 2) Order array of psi at theta limits
        ptlim = np.sort(ptlim)  # Order psi values at theta limits
        # What if the limit case where it only touches the line ?
        # COMMENTED PERFORMANCE    
        # assert(rem(length(ptlim),2)==0, 'PTLIM should always have pair length')
        # 3) Classifies the limit points as enter_avoid = 1 or enter_allow = 0
        lim_class = np.zeros_like(ptlim)  # Classification of limit points (enter_avoid = 1 or enter_allow = 0)

        for i in range(len(ptlim)):
            # Compute theta value (either -jl or jl)
            tlim = np.arctan2((an * np.sin(ptlim[i]) + bn * np.cos(ptlim[i]) + cn),(ad * np.sin(ptlim[i]) + bd * np.cos(ptlim[i]) + cd))
            dlim = at * np.sin(ptlim[i]) + bt * np.cos(ptlim[i]) + ct
            lim_class[i] = np.sign(tlim) == np.sign(dlim)
        # COMMENTED PERFORMANCE    
        # assert(rem(length(ptlim),2)==0, 'PTLIM should always have pair length')    
        # 3) Classifies the limit points as enter_avoid = 1 or enter_allow = 0
        if lim_class[0] == 1:
                #print('In TanJoint, case 3 happend')
                ptlim = np.concatenate(([-np.pi], ptlim, [np.pi]))  # Add -pi and pi to start with enter_allowed

        lim = ptlim
    else:
        #print('In TanJoint, case 4 happend')
        # If no psi values match the joint limits border: either no solutions
        # are allowed or all solutions are allowed.
        # So we just need to check the value of any point contained in the
        # function to know if the codomain is within or outside joint limits
        psi = 0
        tlim = np.arctan2((an * np.sin(psi) + bn * np.cos(psi) + cn),
                            (ad * np.sin(psi) + bd * np.cos(psi) + cd))

        if tlim > -jl and tlim < jl:
                #print('In TanJoint, case 5 happend')
                lim = np.array([-np.pi, np.pi])  # All interval is possible
        else:
                #print('In TanJoint, case 6 happend')
                lim = np.nan  # No interval possible

    if len(sing_lim) > 0:
        #print('In TanJoint, case 7 happend')
        #logger.info(f'lim = {lim} and sing_lim is {np.rad2deg(sing_lim)}')
        lim = IntersectIntervals(lim, sing_lim)  # Intersect with singular limit intervals

    return lim
    
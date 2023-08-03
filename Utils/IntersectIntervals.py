import portion as I
#import intervals as I
import numpy as np
import logging

# Configure the logging settings
logging.basicConfig(format='File Name: %(name)s : %(message)s',
                    level=logging.INFO)

# Get a logger for the current script
logger = logging.getLogger(__name__)


def IntersectIntervals(interval1, interval2):

    # remove NaNs
    #interval1 = interval1[~np.isnan(interval1)]
    #interval2 = interval2[~np.isnan(interval2)]
    #logger.info(f'interval 1 is: {interval1} and interval 2 is: {interval2}')
    n1 = len(interval1)
  

    for i in range(0,n1,2):
        lower_limit = interval1[i]
        upper_limit = interval1[i+1]
        #print(f'step {i} where lower limit is: {lower_limit} and upper limit is {upper_limit}')
        if i == 0:
            interval1_intervals = I.closed(lower_limit,upper_limit)
        else:
            interval1_intervals = interval1_intervals | I.closed(lower_limit,upper_limit)
        

    n2 = len(interval2)
    for i in range(0,n2,2):
        lower_limit = interval2[i]
        upper_limit = interval2[i+1]
        if i == 0:
            interval2_intervals = I.closed(lower_limit,upper_limit)
        else:
            interval2_intervals = interval2_intervals | I.closed(lower_limit,upper_limit)


    # aux = np.concatenate((interval1, interval2))
    # n = int(aux.shape[0] / 2) # number of intervals

    intersect_interval = interval1_intervals & interval2_intervals
    n = len(intersect_interval)
    #print(interval1_intervals)
    intersect_interval_np = np.array([])

    for i in range(n):
        lower_limit = intersect_interval[i].lower
        upper_limit = intersect_interval[i].upper
        intersect_interval_np = np.append(intersect_interval_np,[lower_limit,upper_limit])

    #intersect_interval = np.array([intersect_interval.lower,intersect_interval.upper])
    return intersect_interval_np


# #### Testing the Function

# t1 = np.array([-180,-44.629,-27.875,180])
# t2 = np.array([-62.733,62.733])
# t3 = np.array([-89.286,89.286])
# t4 = np.array([-180,180])
# t5 = np.array([-145.538,82.69])
# t6 = np.array([-87.750,24.902])
# t7 = np.array([-180,3.472,133.540,180])

# t12 = IntersectIntervals(t1,t2)
# #print(t12)
# t34 = IntersectIntervals(t3,t4)
# #print(t34)
# t1234 = IntersectIntervals(t12,t34)
# #print(t1234)
# t56 = IntersectIntervals(t5,t6)
# #print(t56)
# t123456 = IntersectIntervals(t1234,t56)
# #print(t123456)
# t = IntersectIntervals(t123456,t7)

# print(t)
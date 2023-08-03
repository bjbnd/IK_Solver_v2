import numpy as np

def anglemod(angle):
    angle = np.asarray(angle)
    wrapped_angle = np.mod(angle + np.pi, 2 * np.pi) - np.pi
    return wrapped_angle

def CheckSolution(a,b,c,jl,opt,psi):
    tol = 1e-6
    if opt == 1: #tan
        theta = np.arctan2(a[0] * np.sin(psi) + b[0]*np.cos(psi)+c[0],a[1]*np.sin(psi)+b[1]*np.cos(psi)+c[1])
        correct = np.abs(jl-theta) < tol
    else: #cos
        theta = np.arccos(a*np.sin(psi)+b*np.cos(psi)+c)
        correct = np.abs(jl-theta) < tol
    return correct

def ArmAngleFunction( a, b, c, theta, opt):
    #  RMANGLEJOINTANGLE
    #  Solving equation 23 or 24 in order of psi
    #  If theta cannot be reached with any value of psi [-pi, pi]
    #  Returns nothing 
    #  Else
    #  Returns 1 or 2 solutions 
    #  If there is no solution for the psi(theta), means that no value of psi
    #  reaches the theta angle relative to the joint limits 
    

    # a = round(a,6);
    # b = round(b,6);
    # c = round(c,6);
    sol = np.array([0 ,0])
    out = np.array([])
    #  TAN function
    #  We need to work with atan2 because most of the time the theta we are
    #  working with are outside the 1st or 4th quadrant.
    if opt==1:
        v = np.tan(theta)
        a_s = v * (c[1]-b[1]) + (b[0]-c[0])
        b_s = v * 2*a[1] - 2*a[0]
        c_s = v * (b[1]+c[1]) - (b[0]+c[0])

        if b_s**2 - 4*a_s*c_s >= 0:
            sol[0] = 2*np.arctan2(-(b_s -(np.square(b_s**2 - 4*a_s*c_s))),2*a_s)
            sol[1] = 2*np.arctan2(-(b_s +(np.square(b_s**2 - 4*a_s*c_s))),2*a_s)

            # solution in the -pi pi range
            sol = anglemod(sol)

            if CheckSolution(a,b,c,theta,opt,sol[0]):
                out = np.append(out,anglemod(sol[0]))

            if CheckSolution(a,b,c,theta,opt,sol[1]):
                out = np.append(out,anglemod(sol[1]))
    else:
        v = np.cos(theta)
        a_s = v + b - c
        b_s = -2 * a
        c_s = v - b - c

        if b_s**2 - 4*a_s*c_s >= 0:
            sol[0] = 2*np.arctan2(-(b_s -(np.square(b_s**2 - 4*a_s*c_s))),2*a_s)
            sol[1] = 2*np.arctan2(-(b_s +(np.square(b_s**2 - 4*a_s*c_s))),2*a_s)

            # solution in the -pi pi range
            sol = anglemod(sol)
            
            if CheckSolution(a,b,c,theta,opt,sol[0]):
                out = np.append(out,anglemod(sol[0]))

            if CheckSolution(a,b,c,theta,opt,sol[1]):
                out = np.append(out,anglemod(sol[1]))

    return out
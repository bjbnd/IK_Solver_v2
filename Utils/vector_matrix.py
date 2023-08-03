import numpy as np
def vector_matrix(vector):
    x = vector[0]
    y = vector[1]
    z = vector[2]
    first_row = [np.square(x),x*y,x*z]
    second_row =[x*y,np.square(y),y*z]
    third_row = [x*z,y*z,np.square(z)]
    return np.array([first_row,second_row,third_row])
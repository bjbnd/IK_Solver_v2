import numpy as np  

def dh_calc(a,alpha,d,theta):
    t = theta
    a = alpha
    first_row = [np.cos(t),-np.sin(t)*np.cos(a),np.sin(t)*np.sin(a),a*np.cos(t)]
    second_row =[np.sin(t),np.cos(t)*np.cos(a),-np.cos(t)*np.sin(a),a*np.sin(t)]
    third_row = [0,np.sin(a),np.cos(a),d]
    forth_row = [0,0,0,1]
    return np.array([first_row,second_row,third_row,forth_row])
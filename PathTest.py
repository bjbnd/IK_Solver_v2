import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#### Ploting the 3D path
def Plot_Path(x,y,z):
    ## x, y, z are a list which the trajectory points are stroed in that.

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the 3D path
    ax.plot(x, y, z, marker='o')

    # Add labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Path Plot')

    # Show the plot
    plt.show()

    pass

#### Cubic Trajectory
def cubic_trajectory(t,q0,qf):
    t = t/5
    joint_value = q0 + 3*(qf-q0)*t**2 - 2*(qf-q0)*t**3
    return joint_value

############ Real_time control parameteres
time_duration = 5
control_frequency = 20
num_points = int(time_duration * control_frequency)
time_range = np.linspace(0,time_duration,num_points)

### robot parameteres
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
lbs = l[0]
lse = l[1]  # upper arm length (shoulder to elbow)
lew = l[2]  # lower arm length (elbow to wrist)

##### desired pose
pose = np.array([[0,0,1,0.5],
              [0,1,0,0.3],
              [-1,0,0,0.8],
              [0,0,0,1]])

##### Generating a cubic trajectory for each position cordinate
x_traj = []
y_traj = []
z_traj = []
#### Enter the initial position here:
pos_x = 0.4
pos_y = 0
pos_z = 0.720
for t in time_range:
    temp_x = cubic_trajectory(t,pos_x,pose[0,3])
    temp_y = cubic_trajectory(t,pos_y,pose[1,3])
    temp_z = cubic_trajectory(t,pos_z,pose[2,3])

    x_traj.append(temp_x)
    y_traj.append(temp_y)
    z_traj.append(temp_z)

    pos_x = temp_x
    pos_y = temp_y
    pos_z = temp_z

n = len(x_traj)
results = []
for i in range(n):

    x = x_traj[i]
    y = y_traj[i]
    z = z_traj[i]
    pose = np.array([[0,0,1,x],
              [0,1,0,y],
              [-1,0,0,z],
              [0,0,0,1]])
    xend = pose[:3, 3]  # end-effector position from base    
    xs0 = np.array([0, 0, dh[0, 2]])  # shoulder position from base 
    xwt = np.array([0, 0, dh[-1, 2]])  # end-effector position from wrist
    xw0 = xend - np.dot(pose[:3, :3], xwt)  # wrist position from base
    xsw = xw0 - xs0  # shoulder to wrist vector

    result_1 = np.linalg.norm(xsw) < lse + lew and np.linalg.norm(xsw) > lse - lew
    result_2 = abs((np.linalg.norm(xsw)**2 - lse**2 - lew**2) - (2*lse*lew)) > tol
    results.append([x, y, z, result_1, result_2])

# Save results to a CSV file
output_file = "results.csv"
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["X", "Y", "Z", "Condition1_Result", "Condition2_Result"])
    writer.writerows(results)

print("Results saved to:", output_file)

Plot_Path(x_traj,y_traj,z_traj)
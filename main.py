from BuggySimulator import *
import numpy as np
#from controller_slow import *
from controller_fast import *
from util import *
import matplotlib.pyplot as plt
from Evaluation import *

# get the trajectory
traj = get_trajectory('buggyTrace.csv')
# initial the Buggy
vehicle = initail(traj, 0)
n = 6000
X = []
Y = []
delta = []
xd = []
yd = []
phi = []
phid = []
deltad = []
F = []
minDist =[]
'''
your code starts here
'''
# preprocess the trajectory
passMiddlePoint = False
nearGoal = False
for i in range(n):
    print(i)
    command = controller(traj, vehicle.state)
    vehicle.update(command = command)

    # termination check
    disError,nearIdx = closest_node(vehicle.state.X, vehicle.state.Y, traj)
    stepToMiddle = nearIdx - len(traj)/2.0
    if abs(stepToMiddle) < 100.0:
        passMiddlePoint = True
        print('middle point passed')
    nearGoal = nearIdx >= len(traj)-50
    if nearGoal and passMiddlePoint:
        print('destination reached!')
        break
    # record states
    X.append(vehicle.state.X)
    Y.append(vehicle.state.Y)
    delta.append(vehicle.state.delta)
    xd.append(vehicle.state.xd)
    yd.append(vehicle.state.yd)
    phid.append(vehicle.state.phid)
    phi.append(vehicle.state.phi)
    deltad.append(command.deltad)
    F.append(command.F)
    minDist.append(disError)
    
    # to save the current states into a matrix
    currentState = vehicle.state
    cur_state_np = np.array([
                [currentState.xd,
                 currentState.yd,
                 currentState.phid,
                 currentState.delta,
                 currentState.X,
                 currentState.Y,
                 currentState.phi]])
    if i == 0:
        state_saved = cur_state_np.reshape((1, 7))
    else:
        state_saved = np.concatenate((state_saved, cur_state_np.reshape((1, 7))), axis=0)

showResult(traj,X,Y,delta,xd,yd,F,phi,phid,minDist)

np.save('Actual_trajectory', state_saved)
evaluation(minDist, traj, X, Y,4)

plt.title('Trajectory of the Car')
plt.plot(traj[:,0], traj[:,1], 'b', label = "reference trajectory")
plt.plot(X[:],Y[:],'r', label = "actual trajectory")
plt.xlabel("X-position (m)")
plt.ylabel("Y-position (m)")
plt.legend()
plt.show()
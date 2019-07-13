# please do not change this file
import numpy as np
import matplotlib.pyplot as plt
from BuggySimulator import *


def wrap2pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# initial your buggy
def initail(traj,n):
    v = vehicle(vehicle.state(X=traj[n, 0],
                              Y=traj[n, 1],
                              xd=0.1,
                              phi=np.arctan2(traj[n+1, 1] - traj[n, 1], traj[n+1, 0] - traj[n, 0])
                              ))
    return v


# vectorized find_nearest point
def closest_node(X, Y, traj):
    point = np.array([X, Y])
    traj = np.asarray(traj)
    dist = point - traj
    dist_2 = np.sum(dist ** 2, axis=1)
    minIndex = np.argmin(dist_2)
    return np.sqrt(dist_2[minIndex]), minIndex


# find the nearest point on the trajectory
def find_nearest_points(X, Y, traj):
    dist_sq = np.zeros(traj.shape[0])
    for j in range(traj.shape[0]):
        dist_sq[j] = (traj[j,0] - X)**2 + (traj[j,1] - Y)**2
    minDistSqure, minIdx = min((dist_sq[i], i) for i in range(len(dist_sq)))
    return np.sqrt(minDistSqure), minIdx


# get the trajectory from .csv file
def get_trajectory(filename):
    with open(filename) as f:
        lines = f.readlines()
        traj = np.zeros((len(lines), 2))
        for idx, line in enumerate(lines):
            x = line.split(",")
            traj[idx, 0] = x[0]
            traj[idx, 1] = x[1]
    return traj


# save the states
def save_state(currentState):
    cur_state = [currentState.xd,
                 currentState.yd,
                 currentState.phid,
                 currentState.delta,
                 currentState.X,
                 currentState.Y,
                 currentState.phi]
    return cur_state


# show result
def showResult(traj,X,Y,delta,xd,yd,F,phi,phid,minDist):
    print('total steps: ', 0.05*len(X))

    fig, axes = plt.subplots(nrows=4, ncols=2)
    fig.tight_layout()

    plt.subplot(421)
    plt.title('position')
    plt.plot(traj[:, 0], traj[:, 1], 'b')
    plt.plot(X, Y, 'r')

    plt.subplot(422)
    plt.title('delta')
    plt.plot(delta, 'r')

    plt.subplot(423)
    plt.title('xd')
    plt.plot(xd, 'r')

    plt.subplot(424)
    plt.title('yd')
    plt.plot(yd, 'r')

    plt.subplot(425)
    plt.title('phi')
    plt.plot(phi, 'r')

    plt.subplot(426)
    plt.title('phid')
    plt.plot(phid, 'r')

    plt.subplot(427)
    plt.title('minDist')
    plt.plot(minDist, 'r')

    plt.subplot(428)
    plt.title('F')
    plt.plot(F, 'r')

    avgDist = sum(minDist) / len(minDist)
    print('maxMinDist: ', max(minDist))
    print('avgMinDist: ', avgDist)
    plt.show()



if __name__ == "__main__":
    pass
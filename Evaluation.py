import numpy as np
from numpy import linalg as LA
import math
from util import *


def dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def clGrader(traj, X, Y, fs, Cmax_cl):
    ng = 0.0
    ntrack= traj.shape[0]
    XY = np.array([X,Y])
    XY = XY.T
    for i in range(ntrack):
        minDist,_ = closest_node(traj[i,0], traj[i, 1], XY)
        if minDist <= Cmax_cl:
            ng += 1
        else:
            print(i)
    return fs * (ng / float(ntrack))


def adGrader(minDistList, fs, Cavg):
    avg = np.average(minDistList)
    if avg <= Cavg:
        return fs
    if avg <= Cavg*2:
        return (-20/Cavg)*avg + 40
    return 0


def mdGrader(minDistList, fs, Cmax_md):
    ng = 0
    for i in range(len(minDistList)):
        if minDistList[i] <= Cmax_md:
            ng += 1
    return fs*ng/len(minDistList)


def beatBaselineGrader(timeCurrent, timeBaseline):
    if timeCurrent <= timeBaseline:
        return 10
    elif timeCurrent <= 2.0*timeBaseline:
        return 20 - 10*timeCurrent/timeBaseline
    else:
        return 0


def evaluation(minDistList, traj_, X, Y, taskNum):
    print('evaluating...')
    timeBaseline = 250
    dt = 0.05
    Cmax_cl = 6.0 			# constrain of maximun distance for completing the loop
    Cavg = 3.0 				# constrain of average distance
    Cmax_md = 6.0 			# constrain of maximun distance
    fs = 20.0 				# the full score you can get
    traj = traj_[1:len(traj_)-60,:]
    comGrad = clGrader(traj, X, Y, fs, Cmax_cl) 	# grade if you complete the loop
    if taskNum == 2:
        print('Score for complete the loop:', comGrad)
        avgGrad = adGrader(minDistList, fs, Cavg)					# grade your average distance
        print('Score for average distance:', avgGrad)
        maxGrad = mdGrader(minDistList, fs, Cmax_md)				# grade your maximum distance
        print('Score for maximum distance:', maxGrad)
        grade = comGrad + avgGrad + maxGrad
        print('Total score for task 4.2: ', grade)
        return grade
    else:
        if comGrad < fs:
            print('your vehicle did not finish the loop '
                  '\n you cannot enter competition'
                  '\nTotal score for task 4.3 and 4.4:', 0)
            return
        else:
            timeCurrent = len(X) * dt
            beatBaselineScore = beatBaselineGrader(timeCurrent, timeBaseline)
            print('Your time is ',
                  timeCurrent,
                  '\nTotal score for task 4.3: ', beatBaselineScore)
            return beatBaselineScore
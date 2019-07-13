import numpy as np
from scipy import signal
import BuggySimulator
import scipy
from util import *

def controller(traj, currentState):
	#     controller
    
    ''' parameters '''
    lr = 1.7
    lf = 1.1
    Ca = 15000.0
    Iz = 3344.0
    f = 0.01
    m = 2000.0
    g = 10
    dt = 0.05
        
    ''' filter the track '''
    if 'smooth_traj_x' not in controller.__dict__:
        x = traj[:,0]
        y = traj[:,1]
        sig = 5
        controller.smooth_traj_x = scipy.ndimage.gaussian_filter1d(x,sigma = sig,order = 0, mode = 'reflect')
        controller.smooth_traj_y = scipy.ndimage.gaussian_filter1d(y,sigma = sig,order = 0, mode = 'reflect')
        
    ''' calculate the derivative of track '''
    if 'dx' not in controller.__dict__:
        x = traj[:,0]
        y = traj[:,1]
        sig = 5
        controller.dx = scipy.ndimage.gaussian_filter1d(x,sigma = sig,order = 1, mode = 'reflect')
        controller.dy = scipy.ndimage.gaussian_filter1d(y,sigma = sig,order = 1, mode = 'reflect')

    ''' calculate the curvature of track '''
    if 'curvature' not in controller.__dict__:
        x = traj[:,0]
        y = traj[:,1]
        sig = 5
        dxdt = scipy.ndimage.gaussian_filter1d(x,sigma = sig,order = 1, mode = 'wrap')
        d2xdt2 = scipy.ndimage.gaussian_filter1d(dxdt,sigma = sig,order = 2, mode = 'wrap')
        dydt = scipy.ndimage.gaussian_filter1d(y,sigma = sig,order = 1, mode = 'wrap')
        d2ydt2 = scipy.ndimage.gaussian_filter1d(dydt,sigma= sig,order = 2, mode = 'wrap')
        controller.curvature = np.abs(dxdt*d2ydt2-dydt*d2xdt2)/np.power(dxdt**2+dydt**2,3.0/2.0)

    ''' vehicle model '''
    Vx = currentState.xd
    if Vx < 0.1:
        Vx = 0.1
    A = np.array([[0,1,0,0],[0,-4*Ca/m/Vx,4*Ca/m,2*Ca*(lr-lf)/m/Vx],
                  [0,0,0,1],[0,-2*Ca*(lf-lr)/Iz/Vx,2*Ca*(lf-lr)/Iz,-2*Ca*(lf*lf+lr*lr)/Iz/Vx]])
    B1 = np.array([[0],[2*Ca/m],[0],[2*Ca*lf/Iz]])
    B2 = np.array([[0],[-2*Ca*(lf-lr)/m/Vx - Vx],[0],[-2*Ca*(lf*lf+lr*lr)/Iz/Vx]])
    
    ''' descretize the system '''
    C = np.zeros((1,4))
    D = 0
    sys = scipy.signal.cont2discrete((A,B1,C,D),dt)
    Ad = sys[0]
    Bd = sys[1]
    
    ''' find points on the track closest to the car '''
    min_dis_sq = 1000000000;
    min_index = 0;
    for j in range(traj.shape[0]):
        distance_sq = (controller.smooth_traj_x[j] - currentState.X)**2 + (controller.smooth_traj_y[j] - currentState.Y)**2
        if distance_sq <min_dis_sq:
            min_dis_sq = distance_sq
            min_index = j
    min_dis = min_dis_sq**(1/2)
    
    ''' look at a target points 20m away from the current position '''
    if min_index < 7300:
        future = 180
    else:
        future = 155
    
    target_index = min_index+future
    if target_index >= traj.shape[0]:
        target_index = -1
    next_index = min_index+1
    if next_index >= traj.shape[0]:
        next_index =-1
    
    ''' find the current distance from car to the road '''
    road_point1 = np.array([[controller.smooth_traj_x[min_index-1],controller.smooth_traj_y[min_index-1]]])
    road_point2 = np.array([[controller.smooth_traj_x[next_index],controller.smooth_traj_y[next_index]]])
    car = np.array([[currentState.X,currentState.Y]])
    e1 = np.cross(road_point2-road_point1,car-road_point1)/np.linalg.norm(road_point2-road_point1)
    
    ''' find desired road angle '''
    road_angle_x = controller.dx[target_index]
    road_angle_y = controller.dy[target_index]
    desired_road_angle = np.arctan2(road_angle_y,road_angle_x)
    if (desired_road_angle <0):
        desired_road_angle += 2*np.pi
    
    ''' find estimated angle error e2'''
    actual_angle = currentState.phi
    if(actual_angle < 0):
       actual_angle += 2*np.pi
    e2 = actual_angle - desired_road_angle
    if (e2 > np.pi):
        e2 -= 2*np.pi
    elif (e2 < -np.pi):
        e2 += 2*np.pi
    
    ''' find de1 '''
    de1 = currentState.yd + Vx*(e2)
    
    ''' find de2 '''
    desired_road_angle_rate = Vx*controller.curvature[target_index]
    de2 = currentState.phid - desired_road_angle_rate

    ''' find longitudinal speed depending on road angle '''
    max_speed = 8;
    desired_speed = max_speed
#    desired_speed = max_speed/3 + max_speed *2/3* np.cos(desired_road_angle_rate*dt);
    
    ''' use PID to control longitudinal speed '''
    gain = 10000;
    F = gain*(desired_speed - currentState.xd)
    
    ''' find k '''
# ================== infinite horizen LQR ======================== 
    Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    R = 9
    s = np.matrix(scipy.linalg.solve_discrete_are(Ad, Bd, Q, R))
    k = scipy.linalg.inv(Bd.T.dot(s).dot(Bd) + R).dot(Bd.T.dot(s).dot(Ad))
#    eigval,eigvec = scipy.linalg.eig(Ad-Bd.dot(k))
#    print(np.abs(eigval))
    k = -k
    
    ''' output u = kx '''
    estimated_x = np.array([[e1],[de1],[e2],[de2]])
    delta = k.dot(estimated_x)
    delta_d = (delta - currentState.delta)/dt
    delta_d = float(delta_d)

    result = BuggySimulator.vehicle.command(F,delta_d)
    return result







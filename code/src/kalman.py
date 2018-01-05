#!/usr/bin/env python  
import roslib
roslib.load_manifest('marina')
import rospy
import math
import numpy
import tf
import time
import matplotlib.pyplot as plt
from tf import transformations as t

from marina.msg import error_msg

import tf2_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

global file2
file2 = open('/home/sofia/Documents/info_1.txt', 'w')


def QuaternionMultiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return numpy.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=numpy.float64)


def QuaternionMultiplyROS(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return numpy.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=numpy.float64)


def f(prev_state, dt):
    state = numpy.zeros((20,1))
    #print("f pequeno")
    '''
    q = numpy.zeros((4,1))
    w_norm = [0,0,0]
    norm = math.sqrt(prev_state[10, 0]**2+prev_state[11,0]**2+prev_state[12, 0]**2)    
    w_norm[0] = prev_state[10]/norm
    w_norm[1] = prev_state[11]/norm
    w_norm[2] = prev_state[12]/norm
    #q = math.cos(norm*dt/2)*prev_state[6:10,0]+w_norm[0]*math.sin(norm*dt/2)*prev_state[6:10,0]+w_norm[1]*math.sin(norm*dt/2)*prev_state[6:10,0]+w_norm[2]*math.sin(norm*dt/2)*prev_state[6:10,0]
    '''
    W = numpy.array([   [1, -prev_state[10,0]*dt/2, -prev_state[11,0]*dt/2, -prev_state[12,0]*dt/2], 
                        [prev_state[10,0]*dt/2, 1, -prev_state[12,0]*dt/2, prev_state[11,0]*dt/2],
                        [prev_state[11,0]*dt/2, prev_state[12,0]*dt/2, 1, -prev_state[10,0]*dt/2],
                        [prev_state[12,0]*dt/2, -prev_state[11,0]*dt/2, prev_state[10,0]*dt/2, 1]] ) 
    #print(W)
    state[0:3,0] = prev_state[0:3,0] + dt*prev_state[3:6,0]
    state[3:6,0] = (state[0:3,0]-prev_state[0:3,0])/dt  

    state[6:10,0] = numpy.dot(W,prev_state[6:10,0]) - prev_state[16:,0]
    norm = math.sqrt(state[6,0]**2+state[7,0]**2+state[8,0]**2+state[9,0]**2)
    #print("norm", norm)
    state[6:10,0] = state[6:10,0]/norm  

    state[10:13,0] = prev_state[10:13,0]
    state[13:16,0] = prev_state[13:16,0]
    state[16:,:] = prev_state[16:,:]    

    
    return state


def h(curr_state, dt, n_valid_markers):
    #print("h pequeno")
    z_est = numpy.zeros((7+11*n_valid_markers,1))

    if n_valid_markers == 0:
        z_est[0,0]   = (curr_state[6,0]**2+curr_state[7,0]**2-curr_state[8,0]**2-curr_state[9,0]**2)*curr_state[10,0] + (2*curr_state[7,0]*curr_state[8,0]-2*curr_state[6,0]*curr_state[9,0])*curr_state[11,0] + (2*curr_state[6,0]*curr_state[8,0]+2*curr_state[9,0]*curr_state[7,0])*curr_state[12,0]+curr_state[13,0]
        z_est[1,0]   = (2*curr_state[7,0]*curr_state[8,0]+2*curr_state[6,0]*curr_state[9,0])*curr_state[10,0] + (curr_state[6,0]**2-curr_state[7,0]**2+curr_state[8,0]**2-curr_state[9,0]**2)*curr_state[11,0] + (2*curr_state[8,0]*curr_state[9,0]-2*curr_state[6,0]*curr_state[7,0])*curr_state[12,0]+curr_state[14,0]
        z_est[2,0]   = (2*curr_state[9,0]*curr_state[7,0]-2*curr_state[6,0]*curr_state[8,0])*curr_state[10,0] + (2*curr_state[6,0]*curr_state[7,0]+2*curr_state[9,0]*curr_state[8,0])*curr_state[11,0] + (curr_state[6,0]**2-curr_state[7,0]**2-curr_state[8,0]**2+curr_state[9,0]**2)*curr_state[12,0]+curr_state[15,0]
        z_est[3:7,:] = curr_state[16:,:]

    else:
        m = 0
        for i in range(n_valid_markers):
            i_qb = z_est.shape[0]-(n_valid_markers-i)*4
            z_est[m:m+3,0]   = curr_state[0:3,0]
            z_est[m+3:m+7,0] = curr_state[6:10,0]
            z_est[i_qb:i_qb+4,0] = curr_state[16:,0]
            m += 7
    
        z_est[m,0]       = (curr_state[6,0]**2+curr_state[7,0]**2-curr_state[8,0]**2-curr_state[9,0]**2)*curr_state[10,0] + (2*curr_state[7,0]*curr_state[8,0]-2*curr_state[6,0]*curr_state[9,0])*curr_state[11,0] + (2*curr_state[6,0]*curr_state[8,0]+2*curr_state[9,0]*curr_state[7,0])*curr_state[12,0]+curr_state[13,0]
        z_est[m+1,0]     = (2*curr_state[7,0]*curr_state[8,0]+2*curr_state[6,0]*curr_state[9,0])*curr_state[10,0] + (curr_state[6,0]**2-curr_state[7,0]**2+curr_state[8,0]**2-curr_state[9,0]**2)*curr_state[11,0] + (2*curr_state[8,0]*curr_state[9,0]-2*curr_state[6,0]*curr_state[7,0])*curr_state[12,0]+curr_state[14,0]
        z_est[m+2,0]     = (2*curr_state[9,0]*curr_state[7,0]-2*curr_state[6,0]*curr_state[8,0])*curr_state[10,0] + (2*curr_state[6,0]*curr_state[7,0]+2*curr_state[9,0]*curr_state[8,0])*curr_state[11,0] + (curr_state[6,0]**2-curr_state[7,0]**2-curr_state[8,0]**2+curr_state[9,0]**2)*curr_state[12,0]+curr_state[15,0]
        z_est[m+3:m+7,:] = curr_state[16:,:]
    
    return z_est


class Kalman:

    # Class variables initialization
    #pydata from markers. If 0 data is not valid
    data_tf=[ [0,[]],[0,[]],[0,[]],[0,[]],[0,[]],[0,[]],[0,[]],[0,[]],[0,[]]]
    #stores last pressure data
    data_pressure=0
    #stores imu last rate data from imu
    data_rate_imu= []
    #stores imu last linear acceleration data from imu
    data_acc_imu = []
    #ground truth
    optitrack=[]

    #distance between markers
    distance= 0.24
    #marker position
    markers_translation = [ [0.0, 0.0, 0.0],                    #marker0
                            [0.0, distance, 0.0],               #marker1
                            [0.0, 2*distance, 0.0],             #marker2
                            [distance, 0.0, 0.0],               #marker3
                            [distance, distance, 0.0],          #marker4
                            [distance, 2*distance, 0.0],        #marker5
                            [2*distance, 0.0, 0.0],             #marker6
                            [2*distance, distance, 0.0],        #marker7
                            [2*distance, 2*distance, 0.0]]      #marker8

    #pub_error_pos = rospy.Publisher('error_pos', std_msgs.msg.Float32, queue_size=10)
    #pub_error_ang = rospy.Publisher('error_ang', std_msgs.msg.Float32, queue_size=10)
    plt.ion()
    plt.show()


    #step
    h=0

    #stores last values of marker attitude               
        #state
    state = numpy.array([ [0.1], [0.1], [0.1], [0], [0], [0], [1], [0], [0], [0],  [0], [0], [0], [0], [0], [0], [0], [0], [0], [0] ] )
    state_update = numpy.array([ [0.1], [0.1], [0.1], [0], [0], [0], [1], [0], [0], [0],  [0], [0], [0], [0], [0], [0], [0], [0], [0], [0] ] )
    state_pred = numpy.array([ [0.1], [0.1], [0.1], [0], [0], [0], [1], [0], [0], [0],  [0], [0], [0],  [0], [0], [0], [0], [0], [0], [0] ] )
    #state transformation matrix

    '''
    U = h * numpy.array([   [h/2*(q[0]**2+q[1]**2-q[2]**2-q[3]**2), h/2*(2*q[1]*q[2]-2*q[0]*q[3]),          h/2*(2*q[0]*q[3]+2*q[3]*q[1])],
                            [h/2*(2*q[1]*q[2]+2*q[0]*q[3]),         h/2*(q[0]**2-q[1]**2+q[2]**2-q[3]**2),  h/2*(2*q[2]*q[3]-2*q[0]*q[1])],
                            [h/2*(2*q[3]*q[1]-2*q[0]*q[2]),         h/2*(2*q[0]*q[1]+2*q[3]*q[2]),           h/2*(q[0]**2-q[1]**2-q[2]**2+q[3]**2)],

                            [q[0]**2+q[1]**2-q[2]**2-q[3]**2,   2*q[1]*q[2]-2*q[0]*q[3],            2*q[0]*q[3]+2*q[3]*q[1]],
                            [2*q[1]*q[2]+2*q[0]*q[3],           q[0]**2-q[1]**2+q[2]**2-q[3]**2,    2*q[2]*q[3]-2*q[0]*q[1]],
                            [2*q[3]*q[1]-2*q[0]*q[2],           2*q[0]*q[1]+2*q[3]*q[2],             q[0]**2-q[1]**2-q[2]**2+q[3]**2],

                            [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0], 

                            [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0],

                            [0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0] ])   
    '''
    gamma = 100

    # Covariance matrix

    Q = 0.01*numpy.identity(20)
    P = numpy.identity(20)

    # Observation matrix
    H_markers = numpy.array([   [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],                                     
                                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] ])


    R_base = numpy.array([  [0.2204, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0.2504, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0.1456, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],

                            [0, 0, 0, 0.0033, 0.0012, -0.002, 0.0011, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0.0012, 0.0152, -0.0094, 0.001, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, -0.002, -0.0094, 0.0217, -0.0015, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0.0011, 0.001, -0.0015, 0.0036, 0, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            
                            [0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0 ,0, 0, 0 ,0 ,0, 0],
                            
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 100, 0 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0, 100 ,0, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0, 0 ,100, 0, 0 ,0 ,0, 0],
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0, 0 ,0, 100, 0 ,0 ,0, 0],

                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0 ,0 ,0, 0, 10, 0 ,0, 0],
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0 ,0 ,0, 0, 0, 10 ,0, 0],
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0 ,0 ,0, 0, 0, 0 ,10, 0],
                            [0, 0, 0, 0 ,0, 0, 0 ,0 ,0, 0, 0 ,0 ,0, 0, 0, 0 ,0, 10] ])

    prev_time = 0
    curr_time=0

    i_valid_markers_current = []
    i_valid_markers_previous = []
    count = 0
    count_camera = 0
    count_matching = 0
    n = 0
    aux_collect = [0,0,0]
    aux_collect2 = [0,0,0]
    markers_on = 0
    c = 100000
    

    @classmethod
    def valid_markers(cls):
        n_valid_markers = 0
        index_valid_markers = list()
        for i in range(len(cls.data_tf)):
            if cls.data_tf[i][0] == 1:
                n_valid_markers += 1
                index_valid_markers.append(i)

        #print("N. Valid markers: ", n_valid_markers)
        #print("Index valid markers: ", index_valid_markers)
        return n_valid_markers, index_valid_markers
    

    @classmethod
    def create_F(cls):  
        #print("F grande")
        F = numpy.zeros((20,20)) 

        F[0:6,:] =    [ [1, 0, 0, cls.h, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                        [0, 1, 0, 0, cls.h, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, cls.h, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ]
        

        F[10:,:] = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                    
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], 
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], 

                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0], 
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] ]
        
        F[6:10,:]  = [  [0, 0, 0, 0, 0, 0, 1, -cls.state[10,0]*cls.h/2, -cls.state[11,0]*cls.h/2, -cls.state[12,0]*cls.h/2, -cls.state[7,0]*cls.h/2, -cls.state[8,0]*cls.h/2, -cls.state[9,0]*cls.h/2, 0, 0, 0, -1, 0, 0, 0], 
                        [0, 0, 0, 0, 0, 0, cls.state[10,0]*cls.h/2, 1, -cls.state[12,0]*cls.h/2, cls.state[11,0]*cls.h/2, cls.state[7,0]*cls.h/2, cls.state[9,0]*cls.h/2, -cls.state[8,0]*cls.h/2, 0, 0, 0, 0, -1, 0, 0],
                        [0, 0, 0, 0, 0, 0, cls.state[11,0]*cls.h/2, cls.state[12,0]*cls.h/2, 1, -cls.state[10,0]*cls.h/2, -cls.state[9,0]*cls.h/2, cls.state[6,0]*cls.h/2, cls.state[7,0]*cls.h/2, 0, 0, 0, 0, 0, -1, 0],
                        [0, 0, 0, 0, 0, 0, cls.state[12,0]*cls.h/2, -cls.state[11,0]*cls.h/2, cls.state[10,0]*cls.h/2, 1, cls.state[8,0]*cls.h/2, -cls.state[7,0]*cls.h/2, cls.state[9,0]*cls.h/2, 0, 0, 0, 0, 0, 0, -1] ]
        
        return F


    @classmethod
    def create_H_R(cls, n_valid_markers):
        #print("Create H & R")

        #State is the state output of the Prediction Step
        if n_valid_markers == 0:
            #print("\t0")
            H = numpy.zeros( (7, cls.H_markers.shape[1]) )
            R = numpy.zeros( (7, 7) )
            H[:,:] = [  [0, 0, 0, 0, 0, 0, 2*cls.state[6,0]*cls.state[10,0]-2*cls.state[9,0]*cls.state[11,0]+2*cls.state[8,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[10,0]+2*cls.state[8,0]*cls.state[11,0]+2*cls.state[9,0]*cls.state[12,0], -2*cls.state[8,0]*cls.state[10,0]+2*cls.state[7,0]*cls.state[11,0]+2*cls.state[6,0]*cls.state[12,0], -2*cls.state[9,0]*cls.state[10,0]-2*cls.state[6,0]*cls.state[11,0]+2*cls.state[7,0]*cls.state[12,0], cls.state[6,0]**2+cls.state[7,0]**2-cls.state[8,0]**2-cls.state[9,0]**2, 2*cls.state[7,0]*cls.state[8,0]-2*cls.state[6,0]*cls.state[9,0], 2*cls.state[7,0]*cls.state[9,0]+2*cls.state[6,0]*cls.state[8,0], 1, 0, 0, 0, 0, 0, 0], 
                        [0, 0, 0, 0, 0, 0, 2*cls.state[9,0]*cls.state[10,0]+2*cls.state[6,0]*cls.state[11,0]-2*cls.state[7,0]*cls.state[12,0], 2*cls.state[8,0]*cls.state[10,0]-2*cls.state[7,0]*cls.state[11,0]-2*cls.state[6,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[10,0]+2*cls.state[8,0]*cls.state[11,0]+2*cls.state[9,0]*cls.state[12,0], 2*cls.state[6,0]*cls.state[10,0]-2*cls.state[9,0]*cls.state[11,0]+2*cls.state[8,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[8,0]+2*cls.state[6,0]*cls.state[9,0], cls.state[6,0]**2-cls.state[7,0]**2+cls.state[8,0]**2-cls.state[9,0]**2, 2*cls.state[8,0]*cls.state[9,0]-2*cls.state[6,0]*cls.state[7,0], 0, 1, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, -2*cls.state[8,0]*cls.state[10,0]+2*cls.state[7,0]*cls.state[11,0]+2*cls.state[6,0]*cls.state[12,0], 2*cls.state[9,0]*cls.state[10,0]+2*cls.state[6,0]*cls.state[11,0]-2*cls.state[7,0]*cls.state[12,0], -2*cls.state[6,0]*cls.state[10,0]+2*cls.state[9,0]*cls.state[11,0]-2*cls.state[8,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[10,0]+2*cls.state[8,0]*cls.state[11,0]+2*cls.state[9,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[9,0]-2*cls.state[6,0]*cls.state[8,0], 2*cls.state[8,0]*cls.state[9,0]+2*cls.state[6,0]*cls.state[7,0], cls.state[6,0]**2-cls.state[7,0]**2-cls.state[8,0]**2+cls.state[9,0]**2, 0, 0, 1, 0, 0, 0, 0], 
        
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] ]
            
            R[:,:] = cls.R_base[7:14,7:14]
        
        else:
            #print("\tNOT 0")
            H = numpy.zeros( (cls.H_markers.shape[0]*n_valid_markers+7, cls.H_markers.shape[1]) )
            R = numpy.zeros((cls.H_markers.shape[0]*n_valid_markers+7,cls.H_markers.shape[0]*n_valid_markers+7))
            m = 0
            for n in range(n_valid_markers):
                aux = H.shape[0]-(n_valid_markers-n)*4
                H[m:m+7,:] = cls.H_markers[0:7,:]
                H[aux:aux+4,:] = cls.H_markers[7:,:]
                R[m:m+7,m:m+7] = cls.R_base[0:7,0:7]
                R[aux:aux+4,aux:aux+4] = cls.R_base[cls.R_base.shape[0]-4:cls.R_base.shape[0], cls.R_base.shape[0]-4:cls.R_base.shape[0]]   
                m += 7

            H[m:m+7,:] = [  [0, 0, 0, 0, 0, 0, 2*cls.state[6,0]*cls.state[10,0]-2*cls.state[9,0]*cls.state[11,0]+2*cls.state[8,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[10,0]+2*cls.state[8,0]*cls.state[11,0]+2*cls.state[9,0]*cls.state[12,0], -2*cls.state[8,0]*cls.state[10,0]+2*cls.state[7,0]*cls.state[11,0]+2*cls.state[6,0]*cls.state[12,0], -2*cls.state[9,0]*cls.state[10,0]-2*cls.state[6,0]*cls.state[11,0]+2*cls.state[7,0]*cls.state[12,0], cls.state[6,0]**2+cls.state[7,0]**2-cls.state[8,0]**2-cls.state[9,0]**2, 2*cls.state[7,0]*cls.state[8,0]-2*cls.state[6,0]*cls.state[9,0], 2*cls.state[7,0]*cls.state[9,0]+2*cls.state[6,0]*cls.state[8,0], 1, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 0, 0, 0, 2*cls.state[9,0]*cls.state[10,0]+2*cls.state[6,0]*cls.state[11,0]-2*cls.state[7,0]*cls.state[12,0], 2*cls.state[8,0]*cls.state[10,0]-2*cls.state[7,0]*cls.state[11,0]-2*cls.state[6,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[10,0]+2*cls.state[8,0]*cls.state[11,0]+2*cls.state[9,0]*cls.state[12,0], 2*cls.state[6,0]*cls.state[10,0]-2*cls.state[9,0]*cls.state[11,0]+2*cls.state[8,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[8,0]+2*cls.state[6,0]*cls.state[9,0], cls.state[6,0]**2-cls.state[7,0]**2+cls.state[8,0]**2-cls.state[9,0]**2, 2*cls.state[8,0]*cls.state[9,0]-2*cls.state[6,0]*cls.state[7,0], 0, 1, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, -2*cls.state[8,0]*cls.state[10,0]+2*cls.state[7,0]*cls.state[11,0]+2*cls.state[6,0]*cls.state[12,0], 2*cls.state[9,0]*cls.state[10,0]+2*cls.state[6,0]*cls.state[11,0]-2*cls.state[7,0]*cls.state[12,0], -2*cls.state[6,0]*cls.state[10,0]+2*cls.state[9,0]*cls.state[11,0]-2*cls.state[8,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[10,0]+2*cls.state[8,0]*cls.state[11,0]+2*cls.state[9,0]*cls.state[12,0], 2*cls.state[7,0]*cls.state[9,0]-2*cls.state[6,0]*cls.state[8,0], 2*cls.state[8,0]*cls.state[9,0]+2*cls.state[6,0]*cls.state[7,0], cls.state[6,0]**2-cls.state[7,0]**2-cls.state[8,0]**2+cls.state[9,0]**2, 0, 0, 1, 0, 0, 0, 0], 
                            
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] ]

            R[m:m+7,m:m+7] = cls.R_base[7:14,7:14]

        #print("\t", "H shape: ", H.shape, "R shape: ", R.shape)

        return H, R


    @classmethod
    def create_Z(cls):
        #print("Create Z")        
        n_valid_markers, index_valid_markers = cls.valid_markers()

        norm = math.sqrt(cls.state[6,0]**2+cls.state[7,0]**2+cls.state[8,0]**2+cls.state[9,0]**2)
        explicit_quat = [cls.state[7,0]/norm,cls.state[8,0]/norm,cls.state[9,0]/norm,cls.state[6,0]/norm]
        [roll, pitch, yaw] = t.euler_from_quaternion(explicit_quat)
        pitch = numpy.arcsin(cls.data_acc_imu[0])
        roll  = numpy.arcsin(cls.data_acc_imu[1])
        quat = t.quaternion_from_euler(roll,pitch,yaw)

        if n_valid_markers != 0:
            Z = numpy.zeros( (11*n_valid_markers+7, 1) )
            #print("\t", "n valid markers: ", n_valid_markers, "indexes: ", index_valid_markers)
            m = 0
            for n in range(n_valid_markers):
                i = index_valid_markers[n]
                aux = Z.shape[0]-(n_valid_markers-n)*4

                #print("\t","Index: ", i, "Aux: ", aux)

                Z[m,0] = cls.data_tf[i][1][0]
                Z[m+1,0] = cls.data_tf[i][1][1]
                Z[m+2,0] = cls.data_tf[i][1][2]
                Z[m+3,0] = cls.data_tf[i][1][3]
                Z[m+4,0] = cls.data_tf[i][1][4]
                Z[m+5,0] = cls.data_tf[i][1][5]
                Z[m+6,0] = cls.data_tf[i][1][6] 

                norm = math.sqrt(Z[m+3,0]**2+Z[m+4,0]**2+Z[m+5,0]**2+Z[m+6,0]**2)
                Z[m+3:m+7,0] = Z[m+3:m+7,0]/norm

                Z[aux:aux+4,0] = cls.state[6:10,0]-Z[m+3:m+7,0]
                norm = math.sqrt(Z[aux,0]**2+Z[aux+1,0]**2+Z[aux+2,0]**2+Z[aux+3,0]**2)
                Z[aux:aux+4,0] = Z[aux:aux+4,0]/norm

                m += 7
     
            Z[m,0] = cls.data_rate_imu[0]
            Z[m+1,0] = cls.data_rate_imu[1]
            Z[m+2,0] = cls.data_rate_imu[2]
            Z[m+3,0] = cls.state[6,0]-quat[0]
            Z[m+4,0] = cls.state[7,0]-quat[1]
            Z[m+5,0] = cls.state[8,0]-quat[2]
            Z[m+6,0] = cls.state[9,0]-quat[3]

            norm = math.sqrt(Z[m+3,0]**2+Z[m+4,0]**2+Z[m+5,0]**2+Z[m+6,0]**2)
            Z[m+3:m+7,0] = Z[m+3:m+7,0]/norm

        else:
            Z = numpy.zeros( (7, 1) )  
            Z[0,0] = cls.data_rate_imu[0]
            Z[1,0] = cls.data_rate_imu[1]
            Z[2,0] = cls.data_rate_imu[2]
            Z[3,0] = cls.state[6,0]-quat[0]
            Z[4,0] = cls.state[7,0]-quat[1]
            Z[5,0] = cls.state[8,0]-quat[2]
            Z[6,0] = cls.state[9,0]-quat[3]

            norm = math.sqrt(Z[3,0]**2+Z[4,0]**2+Z[5,0]**2+Z[6,0]**2)
            Z[3:7,0] = Z[3:7,0]/norm

        for i in range(len(cls.data_tf)):
            cls.data_tf[i][0] = 0

        #print("\t","Z: ", Z)
        return Z, n_valid_markers


    @classmethod
    def Step_controler(cls, msg):
        if cls.count_camera != 0:
            #print("CAMERA 1")
            cls.markers_on = 1
            cls.prev_time = cls.curr_time
            cls.curr_time = time.time()
            cls.h = cls.curr_time - cls.prev_time
            
        else:
            #print("CAMERA 0")
            cls.count_camera = 1
            cls.curr_time = time.time()


    @classmethod
    def TFRecorder(cls,msg):
        #goes through all tf's captured
        for i in range(0,len(msg.transforms)):

            bias_x = 0
            bias_y = 0

            #records messages from markers
            if msg.transforms[i].header.frame_id=="usb_cam": #if exists new data regarding the markers

                #get marker number of the detection
                marker_number=int(msg.transforms[i].child_frame_id[-1])

                if marker_number<=8:

                    #get marker number of the detection
                    trans1=cls.markers_translation[marker_number]

                    #get rotation from marker to usb
                    (trans2,rot2) = listener.lookupTransform('/ar_marker_'+str(marker_number), '/usb_cam',rospy.Time(0))

                    trans3=[trans1[0]+trans2[0]+bias_x,trans1[1]+trans2[1]+bias_y,trans1[2]+trans2[2]]
                    rot3=rot2

                    correction=[numpy.sqrt(2)/2, numpy.sqrt(2)/2, 0, 0]
                    rot3=QuaternionMultiplyROS(rot3,correction) 

                    #correction because optitrack has z axis up numpy.sqrt(2)/2
                    correction=[0, 0, 1,0]
                    rot3=QuaternionMultiplyROS(rot3, correction)

                    norm = math.sqrt(rot3[0]**2+rot3[1]**2+rot3[2]**2+rot3[3]**2)
                    rot3=[rot3[0]/norm, rot3[1]/norm, rot3[2]/norm, rot3[3]/norm]

                    br=tf.TransformBroadcaster()
                    br.sendTransform(trans3,rot3,rospy.Time.now(), '/drone_'+str(marker_number), "origin")

                    cls.data_tf[marker_number][0]=1
                    cls.data_tf[marker_number][1]=list(trans3)
                    cls.data_tf[marker_number][1].append(list(rot3)[3])
                    cls.data_tf[marker_number][1]=cls.data_tf[marker_number][1]+list(rot3)[0:3]



    @classmethod
    def PressureRecorder(cls, msg):
        cls.data_pressure=msg.data


    @classmethod
    def ImuRecorder(cls, msg):
        #print("IMU")
        data_rate = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        data_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        if cls.markers_on == 0:
            #print("IMU 0")
            cls.aux_collect = [cls.aux_collect + data_rate for cls.aux_collect, data_rate in zip(cls.aux_collect, data_rate)]
            cls.aux_collect2 = [cls.aux_collect2 + data_acc for cls.aux_collect2, data_acc in zip(cls.aux_collect2, data_acc)]
            cls.n = cls.n+1
            return False
        else:
            #print("IMU 1")
            if cls.n == 0:
                cls.n = 1
            cls.data_rate_imu = [i/cls.n for i in cls.aux_collect]
            cls.data_acc_imu = [i/cls.n for i in cls.aux_collect2]
            norm = math.sqrt(cls.data_acc_imu[0]**2+cls.data_acc_imu[1]**2+cls.data_acc_imu[2]**2)
            cls.data_acc_imu = [i/norm for i in cls.data_acc_imu]
            cls.n = 0
            cls.aux_collect = [0,0,0]
            cls.markers_on = 0
            return True
        #print(msg.header.stamp)


    @classmethod
    def PredictionStep(cls):
        #print("Predition")
        cls.state_pred[:,:] = cls.state[:,:]
        cls.state = f(cls.state_update, cls.h)
        F = cls.create_F()
        cls.P = numpy.dot(numpy.dot(F, cls.P), F.T)+cls.Q    
        #print("\tState:")
        #print(cls.state)     


    @classmethod
    def MatchingStep(cls):
        #print("Matching")
        cls.Z, n_valid_markers = cls.create_Z()
        cls.Z_est = h(cls.state_pred, cls.h, n_valid_markers)
        cls.H, cls.R = cls.create_H_R(n_valid_markers)

        #print("Sizes:\n")
        #print("\tZ: ", cls.Z.shape, "  Z_est: ", cls.Z_est.shape, "  H: ", cls.H.shape, "  R: ", cls.R.shape)

        cls.S = numpy.dot(numpy.dot(cls.H, cls.P), cls.H.T) + cls.R
        #print("\tS: ", cls.S.shape)

        cls.V = cls.Z - cls.Z_est

        if numpy.dot(numpy.dot(cls.V.T, numpy.linalg.inv(cls.S)), cls.V) <= cls.gamma:
            #print("True")
            return True
        else:
            #print("False")
            return False


    @classmethod
    def UpdateStep(cls):
        #print("Update")
        K = numpy.dot(numpy.dot(cls.P, cls.H.T), numpy.linalg.inv(cls.S) )
        #print("\tK: ", K.shape)
        cls.state = cls.state + numpy.dot(K, cls.V)
        norm = math.sqrt(cls.state[6,0]**2+cls.state[7,0]**2+cls.state[8,0]**2+cls.state[9,0]**2)
        cls.state[6:10,0] = cls.state[6:10,0]/norm
        norm = math.sqrt(cls.state[16,0]**2+cls.state[17,0]**2+cls.state[18,0]**2+cls.state[19,0]**2)
        cls.state[16:20,0] = cls.state[16:20,0]/norm

        cls.P = cls.P - numpy.dot(numpy.dot(K, cls.S), K.T) 
        cls.state_update[:,:] = cls.state[:,:]
        #print("\tState:")
        #print(cls.state) 

    '''******************************************************************************
    # Function create_H_R:
    #   This function...
    ******************************************************************************'''
    @classmethod
    def VisualizeData(cls):

        #listens to last available ground truth data
        try:

            #listens to last available ground truth data
            (pos,rot_optitrack) = listener.lookupTransform('world','crazy_gt', rospy.Time(0)) 
            rot_optitrack=[rot_optitrack[3],rot_optitrack[0],rot_optitrack[1],rot_optitrack[2]]
            
            #calculates attitude error from optitrack to kalman estimation
            error_quat = QuaternionMultiply(t.quaternion_from_matrix(t.inverse_matrix(t.quaternion_matrix(rot_optitrack))),[cls.state[6,0], cls.state[7,0],cls.state[8,0],cls.state[9,0]])

            #calculates position error from optitrack to kalman estimation
            error_pos = [-pos[0]-cls.state[0][0],-pos[1]-cls.state[1][0],pos[2]-cls.state[2][0]]
            
            mod_pos_error = numpy.sqrt(numpy.dot(error_pos ,error_pos))

            #print("pos error", mod_pos_error)

            norm = math.sqrt(error_quat[0]**2+error_quat[1]**2+error_quat[2]**2+error_quat[3]**2)
            mod_att_error = 180*2*numpy.arccos(numpy.absolute(error_quat[3]/(norm)))/numpy.pi

            #print("quat error", mod_att_error,error_quat )

            msg = error_msg()

            msg.error_pos=numpy.float32(mod_pos_error)
            msg.error_att=numpy.float32(mod_att_error)

            pub_error.publish(msg)

            #broadcasting transform for debugging   
            norm = math.sqrt(cls.state[6]**2+cls.state[7]**2+cls.state[8]**2+cls.state[9]**2)
            br=tf.TransformBroadcaster()
            br.sendTransform((cls.state[0], cls.state[1], cls.state[2]), (cls.state[7,0]/norm, cls.state[8,0]/norm,cls.state[9,0]/norm,cls.state[6,0]/norm),rospy.Time.now(), "drone_kalman", "origin")
            
            #broadcasts prediction step for debugging
            '''
            norm2 = math.sqrt(cls.state_pred[6]**2+cls.state_pred[7]**2+cls.state_pred[8]**2+cls.state_pred[9]**2)
            pred = tf.TransformBroadcaster()
            pred.sendTransform((cls.state_pred[0], cls.state_pred[1], cls.state_pred[2]), (cls.state_pred[7,0]/norm2, cls.state_pred[8,0]/norm2,cls.state_pred[9,0]/norm2,cls.state_pred[6,0]/norm2),rospy.Time.now(), "prediction", "origin")
            '''


            #------------------------- NOVO ---------------------------
            info = list()
            info = error_pos
            info.append(mod_pos_error)

            for i in range(len(error_quat)):
                info.append(error_quat[i])

            info.append(mod_att_error)

            n_valid, _ = cls.valid_markers()
            send_info = numpy.zeros((1,10))            
            sensor_info = list()
            
            for i in range(n_valid):
                send_info[0,0] = cls.data_tf[i][1][0]
                send_info[0,1] = cls.data_tf[i][1][1]
                send_info[0,2] = cls.data_tf[i][1][2]
                send_info[0,3] = cls.data_tf[i][1][3]
                send_info[0,4] = cls.data_tf[i][1][4]
                send_info[0,5] = cls.data_tf[i][1][5]
                send_info[0,6] = cls.data_tf[i][1][6]

                error_pos_sensor = [-pos[0]-send_info[0,0],-pos[1]-send_info[0,1],pos[2]-send_info[0,2]]            
                error_quat_sensor = QuaternionMultiply(t.quaternion_from_matrix(t.inverse_matrix(t.quaternion_matrix(rot_optitrack))),[send_info[0,3], send_info[0,4],send_info[0,5],send_info[0,6]])

                mod_pos_error_sensor = numpy.sqrt(numpy.dot(error_pos_sensor ,error_pos_sensor))
                norm = math.sqrt(error_quat_sensor[0]**2+error_quat_sensor[1]**2+error_quat_sensor[2]**2+error_quat_sensor[3]**2)
                mod_att_error_sensor = 180*2*numpy.arccos(numpy.absolute(error_quat_sensor[3]/(norm)))/numpy.pi

                for i in range(len(error_pos_sensor)):
                    sensor_info.append(error_pos_sensor[i])

                sensor_info.append(mod_pos_error_sensor)

                for i in range(len(error_quat_sensor)):
                    sensor_info.append(error_quat_sensor[i])

                sensor_info.append(mod_att_error_sensor)

            info_aux = info + sensor_info
            return info_aux
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("no optitrack data available for comparison")

    


    @classmethod
    def Cycle(cls,msg):

        cls.curr_time_cycle = time.time()
        if cls.ImuRecorder(msg):
            if cls.count != 0:
                #print("CYCLE 1")
                cls.PredictionStep()
                if cls.MatchingStep() == True:
                    cls.UpdateStep()
                else:
                    cls.count_matching += 1
                    print(cls.count_matching)
                    #cls.state[:,:] = cls.state_update[:,:]
            #first iteration    
            else:
                #print("CYCLE 0")        
                cls.PredictionStep()
                cls.Z, n_valid_markers = cls.create_Z()
                cls.H, cls.R = cls.create_H_R(n_valid_markers)
                cls.S = numpy.dot(numpy.dot(cls.H, cls.P), cls.H.T) + cls.R
                cls.Z_est = h(cls.state, cls.h, n_valid_markers)

                cls.V = cls.Z - cls.Z_est
                cls.UpdateStep()
                cls.count = 1
                cls.ini_time = cls.curr_time
            
            #time.time()-cls.curr_time_cycle
            #print("Step: ", cls.h, "Latency: ",time.time()-cls.curr_time_cycle)
            #print("\n")
            #cls.VisualizeData()
            crazy_gt_info = cls.VisualizeData()
            if crazy_gt_info != None:
                crazy_gt_info.append(cls.h)
                numpy.savetxt(file2, numpy.asarray([crazy_gt_info]), fmt="%f", newline='\n')

if __name__ == '__main__':

    #Initialization
    rospy.init_node('kalman')
    pub_error = rospy.Publisher('error', error_msg, queue_size=10)
    listener = tf.TransformListener()
    rate = rospy.Rate(0.5)


    #Interrupts to subscribers 
    rospy.Subscriber('/camera/image_raw',
                      sensor_msgs.msg.Image,
                      Kalman.Step_controler)

    rospy.Subscriber('/tf',
                      tf2_msgs.msg.TFMessage,
                      Kalman.TFRecorder)

    
    rospy.Subscriber('/crazyflie/pressure',
                      std_msgs.msg.Float32,
                      Kalman.PressureRecorder)   

    rospy.Subscriber('/crazyflie/imu',
                      sensor_msgs.msg.Imu,
                      Kalman.Cycle)

    rospy.spin()

    file2.close()
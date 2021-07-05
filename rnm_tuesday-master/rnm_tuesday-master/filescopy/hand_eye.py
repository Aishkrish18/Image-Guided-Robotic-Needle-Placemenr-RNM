#!/usr/bin/env python
import cv2 as cv
import rospy

import multiprocessing
import sys
import glob
import numpy as np
import sys
import cv2 as cv
# from cv2 import solve
import scipy
import scipy.linalg as linalg
from numpy.linalg import lstsq
import sympy

# The hand eye calibration is done to find the calibration of camera with respect to the
# gripper or the end-effector. Traditionally the equation AX = YB solves the equation.
# Reproduced the paper QR24 hand eye optimisation where X and Y are simultaneously solved

class hand_eye:

    def ___init___(self):
        pass

# b2g- base2gripper
# c2t camera to tracking system
# From paper e12 = identity matrix of size 12.
    def calibratehandeye(self, b2g, c2t):
# counts the number of transformations in b2g
        count = len(b2g)
        e12 = np.eye(12, dtype=np.float64)
        e12 = e12 * -1
        z = []
# camera to gripper = c2g
# robot base to checkeboard - g2c
        c2g = np.eye(4, dtype=np.float64)
        g2c = np.eye(4, dtype=np.float64)
# Aw = b , A belongs to 12n x 24 and b belongs to 12x1 vector
        A = np.zeros((count*12, 24), dtype=np.float64)
        B = np.zeros((count*12, 1), dtype=np.float64)
        A_ind = np.zeros((12, 24), dtype = np.float64)
        B_ind = np.zeros((12, 1), dtype = np.float64)

# Repeats the process for all the transformations in count
        for i in range(count):
            # forward kinematics -fk_b2g
            # split into rotation and translation
            fk_b2g = b2g[i]
            Rot_b2g = fk_b2g[0:3, 0:3]
            Trans_b2g = -fk_b2g[0:3, 3:4]
            #print(Trans_b2g)
            # camera extrinsic matrix
            cam_tf = c2t[i]

            #A_ind = np.zeros((12,24), dtype = np.float64)
            #B_ind = np.zeros((12,1), dtype = np.float64)
            # Values assigned according to the paper to A and B Matrix
            for a in range(14):
                for x in range(3):
                    for y in range(3):
                        if a==0:
                            A_ind[x][y] = Rot_b2g[x][y]*cam_tf[0][0]

                        elif a==1:
                            A_ind[x+3][y] = Rot_b2g[x][y]*cam_tf[0][1]

                        elif a==2:
                            A_ind[x+6][y] = Rot_b2g[x][y]*cam_tf[0][2]

                        elif a==3:
                            A_ind[x+9][y] = Rot_b2g[x][y]*cam_tf[0][3]

                        elif a==4:
                            A_ind[x][y+3] = Rot_b2g[x][y]*cam_tf[1][0]

                        elif a==5:
                            A_ind[x+3][y+3] = Rot_b2g[x][y]*cam_tf[1][1]

                        elif a==6:
                            A_ind[x+6][y+3] = Rot_b2g[x][y]*cam_tf[1][2]

                        elif a==7:
                            A_ind[x+9][y+3] = Rot_b2g[x][y]*cam_tf[1][3]

                        elif a==8:
                            A_ind[x][y+6] = Rot_b2g[x][y]*cam_tf[2][0]

                        elif a == 9:
                            A_ind[x+3][y+6] = Rot_b2g[x][y]*cam_tf[2][1]

                        elif a == 10:
                            A_ind[x+6][y+6] = Rot_b2g[x][y]*cam_tf[2][2]

                        elif a == 11:
                            A_ind[x+9][y+6] = Rot_b2g[x][y]*cam_tf[2][3]

                        elif a == 12:
                            A_ind[x+9][y+9] = Rot_b2g[x][y]

            A_ind[0:12, 12:24] = e12
            #print(A_ind)

            for x in range(12):
                for y in range(24):
                    A[(i*12)+x][y] = A_ind[x][y]
            # Assigning values to B matrix
            for y in range(len(Trans_b2g)):
                B_ind[9+y] = Trans_b2g[y]

            for y in range(len(B_ind)):
                B[(i*12)+y] = B_ind[y]

        #print(B)
        #print(A)
        print(A.shape)
        # Calculating the the final transformation using least squares method
        z = lstsq(A, B)
        # Splitting 24x24 into end-effector to camera and robot to checkerboard
        c2g[0][0] = z[0][0]
        c2g[0][1] = z[0][1]
        c2g[0][2] = z[0][2]
        c2g[0][3] = z[0][3]
        c2g[1][0] = z[0][4]
        c2g[1][1] = z[0][5]
        c2g[1][2] = z[0][6]
        c2g[1][3] = z[0][7]
        c2g[2][0] = z[0][8]
        c2g[2][1] = z[0][9]
        c2g[2][2] = z[0][10]
        c2g[2][3] = z[0][11]
        print("End Effector to camera ")
        print(c2g)

        g2c[0][0] = z[0][12]
        g2c[0][1] = z[0][13]
        g2c[0][2] = z[0][14]
        g2c[0][3] = z[0][15]
        g2c[1][0] = z[0][16]
        g2c[1][1] = z[0][17]
        g2c[1][2] = z[0][18]
        g2c[1][3] = z[0][19]
        g2c[2][0] = z[0][20]
        g2c[2][1] = z[0][21]
        g2c[2][2] = z[0][22]
        g2c[2][3] = z[0][23]

        print("RobotBase to Checkerboard")
        print(g2c)


if __name__ == '__main__':
    tf1 = np.array([[0.244948, 0.076194, 0.966538, 0.250202],
                    [0.96417, -0.123872, -0.234584, 0.444379],
                    [0.101854, 0.989368, -0.103804, 0.632892],
                    [0, 0, 0, 1]])
    tf2 = np.array([[0.12073, -0.058858, 0.99094, 0.325769],
                    [0.98973, 0.084132, -0.115588, 0.378392],
                    [-0.076568, 0.994716, 0.06841, 0.547474],
                    [0, 0, 0, 1]])
    tf3 = np.array([[0.245294, -0.107032, 0.963518, 0.224766],
                    [0.961376, 0.154818, -0.227546, 0.30891],
                    [-0.12481, 0.982122, 0.14088, 0.649845],
                    [0, 0, 0, 1]])

    cm1 = np.array([[-0.383438, 0.909039, 0.163165, 0.217122 ],
                    [-0.919777, -0.391844, 0.0215974, 0.131643],
                    [0.0835682, -0.141794, 0.986362, -0.558811],
                    [0, 0, 0, 1]])

    cm2 = np.array([[0.295078, 0.022672, 0.955206, 0.250307],
                    [ 0.95424, -0.057814, -0.29341, 0.328177],
                    [0.04857, 0.998074, -0.038696, 0.590762],
                    [0, 0, 0, 1]])

    cm3 = np.array([[-0.375559, 0.915034, -0.147206, 0.136169],
                    [-0.91599, -0.390655, -0.0913996, -0.00123613],
                    [-0.14114, 0.100514, 0.984874, -0.592354],
                    [0, 0, 0, 1]])
    tf_main = []
    cm_main = []

    tf_main.append(tf1)
    tf_main.append(tf2)
    tf_main.append(tf3)

    cm_main.append(cm1)
    cm_main.append(cm2)
    cm_main.append(cm3)
    abc = hand_eye()

    abc.calibratehandeye(tf_main, cm_main)


















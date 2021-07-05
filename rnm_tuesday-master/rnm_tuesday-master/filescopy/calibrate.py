#!/usr/bin/env python
import cv2 as cv
import rospy
import numpy as np
import math
import glob

ct = 0
checkerboardSize = (8, 5)
c2t_mat = np.eye(4, dtype=np.float64)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 1e-6)

obj_points = []
img_points = []
obj_p = np.zeros((checkerboardSize[0]*checkerboardSize[1], 3), np.float32)
obj_p[:, :2] = np.mgrid[0:checkerboardSize[0], 0:checkerboardSize[1]].T.reshape(-1, 2)

images = glob.glob('/home/rnm/Documents/filter4new/1.jpg')
for i in images:
    img = cv.imread(i)
    img = cv.resize(img, (2048, 1536))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    success, corners = cv.findChessboardCorners(gray, checkerboardSize, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK+
                                                                                cv.CALIB_CB_NORMALIZE_IMAGE)
    print("Here now")
    if success:
        obj_points.append(obj_p)
        corners2 = cv.cornerSubPix(gray, corners, (14, 14), (-1, -1), criteria)
        img_points.append(corners2)
        img = cv.drawChessboardCorners(img, checkerboardSize, corners2, success)
    print(ct)
    cv.imshow('img', img)
    cv.waitKey(1)
cv.destroyAllWindows()

print("Calibrating...")
success, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1], cameraMatrix=None,
                                                               distCoeffs=None, rvecs=None, tvecs=None,
                                                               flags=cv.CALIB_RATIONAL_MODEL, criteria=criteria)
r_vector = np.asarray(rvecs[0], dtype=np.float64)
Rot_Mat = np.zeros((3,3), dtype=np.float64)
print("Rotation matrix of rvec0", cv.Rodrigues(r_vector, Rot_Mat, jacobian=0))
print(Rot_Mat)
print("Camera Calibrated", success)
print("\nCameraMatrix:\n", cameraMatrix)
print("\nDistortion Parameters:\n", dist)
print("\nRotation vectors:\n", rvecs)
print("\nTranslation Vectors:\n", tvecs)
print(tvecs[0][0])
for x in range(3):
    for y in range(3):
        c2t_mat[x][y] = Rot_Mat[x][y]
for x in range(3):
    c2t_mat[x][3] = tvecs[0][x]

print(c2t_mat)

images = glob.glob('/home/rnm/Documents/filter4new/*.jpg')
for i in images:

    img = cv.imread(i)
    h, w = img.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))

    dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv.imwrite("/home/rnm/Documents/unDistort/"+str(ct)+".jpg", dst)
    ct = ct+1

mean_error = 0

for i in range(len(obj_points)):

    img_points2, _ = cv.projectPoints(obj_points[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(img_points[i], img_points2, cv.NORM_L2)/len(img_points2)
    mean_error += error

print("total error: {}".format(mean_error/len(obj_points)))

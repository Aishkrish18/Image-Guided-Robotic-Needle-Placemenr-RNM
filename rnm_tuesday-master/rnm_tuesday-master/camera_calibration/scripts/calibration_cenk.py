import random

import numpy as np
import cv2 as cv
from pathlib import Path

data_path = Path("/home/cenkt/rnm/files/Seminar/calibration/calibration_bag_out/")
image_paths = []

for img_path in data_path.glob("*.jpg"):
    image_paths.append(img_path)

image_paths = sorted(image_paths)


checkerboardSize = (5,8)

# noinspection PyUnresolvedReferences
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # ??

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((checkerboardSize[0]*checkerboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboardSize[0], 0:checkerboardSize[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

for file_name in image_paths:
    img = cv.imread(str(file_name))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, checkerboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, checkerboardSize, corners2, ret)
        #cv.imshow(file_name.stem, img)
        #cv.waitKey(100)
        print(file_name.stem+" successful \r",end="")

cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera Calibrated", ret)
print("\nCameraMatrix:\n", mtx)
print("\nDistortion Parameters:\n", dist)
print("\nRotation vectors:\n", rvecs[0])
print("\nTranslation Vectors:\n", tvecs[0])

img = cv.imread(str(np.random.choice(image_paths)))
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

#cv.namedWindow('Undistorted Image', cv.WINDOW_NORMAL)
#cv.resizeWindow('Undistorted Image', 900, 900)
#cv.imshow('Undistorted Image', dst)
#cv.namedWindow('Image', cv.WINDOW_NORMAL)
#cv.resizeWindow('Image', 900, 900)
#cv.imshow('Image', dst)
#wcv.waitKey(0)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2 as cv
import numpy as np
import time

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


calImg = cv.imread('calibrationTest.jpg')
calVid = cv.VideoCapture(0, cv.CAP_DSHOW)
calVid.set

if(calVid.isOpened() == False):
    print('Unable to Open Camera')
else:
    print("CAmera Opened")


while(True):
    ret, frame = calVid.read()
    # print(ret)

    if ret:
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        cv.imshow('Insert Calibration Board', gray)
        retc , corners = cv.findChessboardCorners(gray, (6,6), None)
        if retc:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv.drawChessboardCorners(frame, (6,6), corners2, ret)
            cv.imshow('Insert Calibration Board', frame)
            # time.sleep(2)
            
            # break
    if cv.waitKey(20) & 0xFF==ord('d'):
        break

calVid.release()
cv.destroyAllWindows()



# gray = cv.cvtColor(calImg, cv.COLOR_BGR2GRAY)
# cv.imshow('Gray', gray)
# # Find the chess board corners
# ret, corners = cv.findChessboardCorners(gray, (), None)

# print(ret)

# cv.waitKey(0)

# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2 as cv
import numpy as np
import time
import pickle as pkl 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
saved = False
calibrated = False

calVid = cv.VideoCapture(0, cv.CAP_DSHOW)
calVid.set

if(calVid.isOpened() == False):
    print('Unable to Open Camera')
else:
    print("CAmera Opened")


while(True):
    ret, frame = calVid.read()
    # print(ret)
    if (not calibrated):
        if ret:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            cv.imshow('Insert Calibration Board', gray)
            retc , corners = cv.findChessboardCorners(gray, (6,6), None)
            if retc:
                calibrated = True
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                cv.drawChessboardCorners(frame, (6,6), corners2, ret)
                cv.imshow('Insert Calibration Board', frame)
                time.sleep(3)
                cv.destroyAllWindows()
                # time.sleep(2)
                
                # break
    
    else:
        if not saved:
            ret, mtx, dist, rvecs, tvecs =todumpcalib =  cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

            h, w = frame.shape[:2]
            newcameramtx, roi = todumpnew = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            saved = True

        # undistort
        dst = cv.undistort(frame.copy(), mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        cv.imshow('Calibrated Result', dst)

        
    

    


    if cv.waitKey(20) & 0xFF==ord('d'):
        with open('calibrate.txt', 'wb') as calib_file:
            pkl.dump( todumpcalib, calib_file)
        with open('newcalibrate.txt', 'wb') as new_file:
            pkl.dump( todumpnew, new_file)     
        break


calVid.release()
cv.destroyAllWindows()



# gray = cv.cvtColor(calImg, cv.COLOR_BGR2GRAY)
# cv.imshow('Gray', gray)
# # Find the chess board corners
# ret, corners = cv.findChessboardCorners(gray, (), None)

# print(ret)

# cv.waitKey(0)

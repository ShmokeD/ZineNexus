# https://www.educative.io/answers/shape-detection-opencv
# https://docs.opencv.org/3.4/d3/d05/tutorial_py_table_of_contents_contours.html

import cv2 as cv
import numpy as np
import time
import pickle as pkl

# with open('calibrate.txt','rb') as calib_file:
#     todumpcalib = pkl.load(calib_file)

# with open('newcalibrate.txt','rb') as roi_file:
#     todumpnew = pkl.load(roi_file)

# ret, mtx, dist, rvecs, tvecs =todumpcalib
# newcameramtx, roi = todumpnew


# Working variables
lastTime = time.time()
Input, Output, Setpoint = 0, 0, 0
ITerm, lastInput = 0, 0
kp, ki, kd = 0, 0, 0
SampleTime = 1  # 1 sec
outMin, outMax = 0, 0
inAuto = False

MANUAL = 0
AUTOMATIC = 1

DIRECT = 0
REVERSE = 1
controllerDirection = DIRECT


def Compute():
    global lastTime, Input, Output, Setpoint, ITerm, lastInput
    global kp, ki, kd, SampleTime, outMin, outMax, inAuto

    if not inAuto:
        return

    now = time.time()
    timeChange = now - lastTime
    if timeChange >= SampleTime:
        # Compute all the working error variables
        error = Setpoint - Input
        ITerm += ki * error
        if ITerm > outMax:
            ITerm = outMax
        elif ITerm < outMin:
            ITerm = outMin

        dInput = Input - lastInput

        # Compute PID Output
        Output = kp * error + ITerm - kd * dInput
        if Output > outMax:
            Output = outMax
        elif Output < outMin:
            Output = outMin

        # Remember some variables for next time
        lastInput = Input
        lastTime = now


def SetTunings(Kp, Ki, Kd):
    global kp, ki, kd, SampleTime, controllerDirection

    if Kp < 0 or Ki < 0 or Kd < 0:
        return

    SampleTimeInSec = SampleTime / 1000.0
    kp = Kp
    ki = Ki * SampleTimeInSec
    kd = Kd / SampleTimeInSec

    if controllerDirection == REVERSE:
        kp = -kp
        ki = -ki
        kd = -kd


def SetSampleTime(NewSampleTime):
    global ki, kd, SampleTime

    if NewSampleTime > 0:
        ratio = NewSampleTime / SampleTime
        ki *= ratio
        kd /= ratio
        SampleTime = NewSampleTime


def SetOutputLimits(Min, Max):
    global Output, ITerm, outMin, outMax

    if Min > Max:
        return

    outMin = Min
    outMax = Max

    if Output > outMax:
        Output = outMax
    elif Output < outMin:
        Output = outMin

    if ITerm > outMax:
        ITerm = outMax
    elif ITerm < outMin:
        ITerm = outMin


def SetMode(Mode):
    global inAuto
    newAuto = (Mode == AUTOMATIC)
    if newAuto != inAuto:
        Initialize()
    inAuto = newAuto


def Initialize():
    global lastInput, ITerm, outMin, outMax

    lastInput = Input
    ITerm = Output
    if ITerm > outMax:
        ITerm = outMax
    elif ITerm < outMin:
        ITerm = outMin


def SetControllerDirection(Direction):
    global controllerDirection
    controllerDirection = Direction


# Example usage:

SetTunings(1, 2, 3)
SetSampleTime(1000)
SetOutputLimits(0, 255)
SetMode(AUTOMATIC)
SetControllerDirection(DIRECT)

    # Call Compute() function periodically to compute PID output
    # Update Input and Setpoint values as needed



def readBarcode(cont):
    output = ""
    widths = {}
    for contour in cont:

        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)

        vertices = len(approx)

        if vertices ==3 or vertices ==4:
            rect = cv.minAreaRect(contour)
            
            (centx, centy), (width, height), angle = rect
            # print(f' {centx} {width}')

            box = cv.boxPoints(rect)
            box = np.intp(box)


            widths[centx] = width

    if(len(widths.values()) == 0):
        return 
    


    maxThick = max(widths.values())
    thickX = 0
    for key, val in widths.items():
        if maxThick == val:
            thickX = key

    thinThick = 0.1* maxThick
    thickThick = 0.2* maxThick

    widths.pop(thickX)

    
    s = list(widths.keys())
    s.sort()
    for widt in s:
        if(abs(widths[widt]/thinThick - 1) < 0.15):
            output += "0"
        elif(abs(widths[widt]/thickThick - 1) < 0.15):
            output += "1"
        else:
            pass
            # print(widths[widt]/maxThick)
        
        # print(f'{widths[widt]}  ', end = '')
    
    # time.sleep(0.1)
    return output

    # print(output)
    
    
def getShapeAndStuff(cont) -> list:

    circlePoints = [] #list(tuple())
    rectPoint = []
    i = 0
    
    for contour in cont:

        #Centroid Detection
        M = cv.moments(contour)
        cx = cy = 0
        if(M['m00'] != 0):
            # print(M)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # print("LOOP")
            # print(f'{cx} {cy}')
            # cv.circle( contours, (cx,cy), 1, (0,255,255),  1 )

        #Polygon Approximation
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        perimeter = cv.arcLength(contour, True)


                        
        cv.drawContours(frame, [approx], 0, (0, 255, 0), 2)
        cv.imshow('Approx', frame)
        #TODO: Find the closest circle of large exnough radius and compute pid against it 
        #TODO: Minimum Area to be decided for threshold

        vertices = len(approx)
        
        if vertices == 4 or vertices == 3:
            i+=1
            rectPoint.append(tuple([cx,cy]))
            
            
        elif vertices >5 :
            if perimeter > 500:
                cv.putText(contours, f'{cx}, {cy} {perimeter}', (cx,cy), 1,1,(255,0,0),1)
                circlePoints.append(tuple([cx,cy]))
                    
        
            
       

    if(i>4):
        ccc, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # Might try using cont instead
        cv.drawContours(frame, ccc, -1, (255,0,0),1)
        cv.imshow('ccc', frame)
        try:
            code = int(readBarcode(cont), 2) 
        except ValueError:
            print('Cant Read')   
    
    circlePoints.sort(key= lambda x:x[1], reverse= True)
    rectPoint.sort(key= lambda x:x[1], reverse= True)
    try:
        if(rectPoint[0][1] >= circlePoints[0][1]):
            return ['Rectangle', code]

        else:
            return ['Circle', (cx,cy)]

    except:
        return ['Circle', midPointBase]
    








set = False






if True:
    # vid = cv.VideoCapture('http://100.103.177.251:8080/video')
    minimumLength = 1000
    vid = cv.VideoCapture(0)
#shape[0] gives height shape[1] gives length
    vid.set

    while True:

        ret, frame = vid.read()
        if ret == True:

            frame = cv.resize(frame, (640,480))
        #     frame = cv.undistort(frame.copy(), mtx, dist, None, newcameramtx)
        # # crop the image
        #     x, y, w, h = roi
        #     frame = frame[y:y+h, x:x+w]

            
            Compute()           
            #Line To Compute PID Against(To be Calibrated)
            midPointBase = (frame.shape[1]//2, 0)
            midPointHeight = (frame.shape[1]//2, frame.shape[0])
            if not set:
                followPoint = midPointBase
                set = True

            ############ Canny Edge Detection ###########
            grayed = cv.cvtColor(frame.copy(), cv.COLOR_BGR2GRAY)
            blurred = cv.GaussianBlur(grayed, (7,7), 0)
            edges = cv.Canny(blurred.copy(), 127,255)
            # cv.imshow('Canny', edges)
            
            _, thresh = cv.threshold(blurred.copy(), 127,255, cv.THRESH_BINARY)
            # cv.imshow('Thresh', thresh)
            cont, _ = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            contours = cv.drawContours(frame.copy(), cont, -1, (0,0,255),1)


            
            pointss = getShapeAndStuff(cont)

            if pointss[0] == 'Circle':
                followPoint = pointss[1]
                

            


            
            

            
            ######## Add the Line at Last To not affect Edge Detection #####   
            cv.line(contours,midPointBase, midPointHeight, (255,255,255), 2)

            cv.putText(contours, f'Following {followPoint}', midPointHeight, 4, 1, (225,225,250), 1)
            cv.imshow('Contours', contours)
            

        if cv.waitKey(20) & 0xFF==ord('d'):
            cv.destroyAllWindows()
            break
        

# https://www.educative.io/answers/shape-detection-opencv
# https://docs.opencv.org/3.4/d3/d05/tutorial_py_table_of_contents_contours.html

import cv2 as cv
import numpy as np
import time
import pickle as pkl
import requests

# with open('calibrate.txt','rb') as calib_file:
#     todumpcalib = pkl.load(calib_file)

# with open('newcalibrate.txt','rb') as roi_file:
#     todumpnew = pkl.load(roi_file)

# ret, mtx, dist, rvecs, tvecs =todumpcalib
# newcameramtx, roi = todumpnew


lastTime = time.time()
Input, Output, Setpoint = 0, 0, 240
ITerm, lastInput = 0, 0
kp, ki, kd = 1, 0, 0
SampleTime = 1  # 1 sec
outMin, outMax = -255,255
inAuto = False

MANUAL = 0
AUTOMATIC = 1

DIRECT = 0
REVERSE = 1

controllerDirection = DIRECT

espOn = True
baseUrl = "http://192.168.137.5:80"
headers={'Content-Type': 'application/x-www-form-urlencoded'}

def Compute():
    global lastTime, Input, Output, Setpoint, ITerm, lastInput
    global kp, ki, kd, SampleTime, outMin, outMax, inAuto


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
        print(f"Error = {error}")
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

        if espOn:
            rotate(Output)


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


def startMoving():
    requests.post(baseUrl+'/start',headers = headers)

def stopMoving():
    requests.post(baseUrl+'/stop', headers=headers)

def rotate(value: int):

    if(abs(value)<=90):
        startMoving()
        return
    
    postValue = 0
    if(value < 0):
        postValue= abs(value) + 255
    
    else:
        postValue = value
        

    requests.post(baseUrl+'/turn', headers= headers, data=f'plain={postValue}')
    print(f'Rotate request with {postValue}')
    

greenCodes = [0,3,6,9,12,15]
redCodes = [1,4,7,10,13]
blueCodes = [2,5,8,11,14] 

def readBarcode(image):

   
        #TODO: Find the closest circled of large exnough radius and compute pid against it 
        #TODO: Minimum Area to be decided for threshold
    cont, hier = cv.findContours(image.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    parentArea = 0
    hier = hier[0]
    count = 0
    childrenCount = {}
    parent  = -1
    childrenCont = []
    for i in range(len(hier)):
        count = 0
        child = hier[i][2]
        
        while child!= -1:
            count += 1
            child = hier[child][2]

        childrenCount[i] = count
 
    
    xArea= {}
    xAreaS ={}
    for i, j in childrenCount.items():
        if j == 4:
            parent = i
            parentContour = cont[i]
    
    if parent != -1:
        nxtChild = hier[parent][2]
        while nxtChild != -1:
            childrenCont.append(cont[nxtChild])
            nxtChild = hier[nxtChild][0]

        parentArea = cv.contourArea(parentContour)
        for con in childrenCont:
            moment = cv.moments(con)

            cx = cy = 0
            if(moment['m00'] != 0):
                cx = int(moment['m10']/moment['m00'])
                cy = int(moment['m01']/moment['m00'])

                xArea[cx] = cv.contourArea(con)

        sortedKeys = sorted(xArea.keys())
        xAreaS= {i:xArea[i] for i in sortedKeys}
        
        
    print(parentArea, xAreaS)
    
    circlePoints = [] #list(tuple())
    rectPoint = []
    i = 0


    
    
def getShapeAndStuff(cont) -> list:


   
    i = 0
    circlePoints = []
    rectPoint = []
    triPoints=[]
    for contour in cont:

        #Centroid Detection
        M = cv.moments(contour)
        cx = cy = 0
        if(M['m00'] != 0):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # cv.circle( contours, (cx,cy), 1, (0,255,255),  1 )

        #Polygon Approximation
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        perimeter = cv.arcLength(contour, True)


                        
        cv.drawContours(frame, [approx], 0, (0, 255, 0), 2)
        cv.imshow('Approx', frame)

        #TODO: Minimum Area to be decided for threshold

        vertices = len(approx)
                   
            
        if 30>=vertices >=5 :
            if perimeter > 500:
                cv.putText(frameShown, f'{cx}, {cy}', (cx,cy), 1,1,(255,0,0),1)
                circlePoints.append(tuple([cx,cy]))
        if vertices == 4 :
            if perimeter>500:
                cv.putText(frameShown,f'{cx},{cy}',(cx,cy),1,1,(255,0,0),1)
                rectPoint.append(tuple([cx,cy]))
        if vertices == 3 :
            if perimeter>500:
                cv.putText(frameShown,f'{cx},{cy}',(cx,cy),1,1,(255,0,0),1)
                triPoints.append(tuple([cx,cy]))
            
       

    
    
    
    circlePoints.sort(key = lambda x:x[1], reverse = True)
    rectPoint.sort(key = lambda x:x[1], reverse = True)
    triPoints.sort(key = lambda x:x[1], reverse = True)
    

    if(len(circlePoints) == len(rectPoint) == len(triPoints) == 0):
        return['None', midPointBase]
    
    elif (len(triPoints) != 0):
        return ['Triangle', triPoints[0]]
    
    elif (len(rectPoint) != 0):
        return ['Rectangle', rectPoint[0]]
    
    else:
        return ['Circle', circlePoints[0]]
  



    

set = False


vid = cv.VideoCapture(0)
minimumLength = 1000

# vid = cv.VideoCapture(0)
#shape[0] gives height shape[1] gives length
vid.set

if espOn:
    startMoving()
while True:

    ret, frame = vid.read()
    if ret == True:
        frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
        
        # frame = cv.resize(frame, (640,480))
        frameShown = frame.copy()
        frameSave = frame.copy()

        hsv = cv.cvtColor(frame.copy(), cv.COLOR_BGR2HSV)
        blurredHsv = cv.GaussianBlur(hsv, (9,9), 0)




###################################### MASKS ################################################
        lowerGreen = np.array([36,25,25])
        upperGreen = np.array([70,255,255])
        greenMask = cv.inRange(blurredHsv, lowerGreen ,upperGreen)


        lowerBlue = np.array([100,50,70])
        upperBlue = np.array([130,255,255])
        blueMask = cv.inRange(blurredHsv, lowerBlue, upperBlue)

        
        # lowerRed = np.array([300,130,0])
        # upperRed = np.array([35,255,255])

                # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 255])
        
        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160,100,20])
        upper2 = np.array([179,255,255])
        
        lower_mask = cv.inRange(blurredHsv, lower1, upper1)
        upper_mask = cv.inRange(blurredHsv, lower2, upper2)

        redMask = lower_mask + upper_mask
###################################### MASKS ################################################




        mask = greenMask
        
        masked =  cv.bitwise_and(blurredHsv, blurredHsv, mask = mask)
                
    #     frame = cv.undistort(frame.copy(), mtx, dist, None, newcameramtx)
    # # crop the frame
    #     x, y, w, h = roi
    #     frame = frame[y:y+h, x:x+w]

        


        #Line To Compute PID Against(To be Calibrated)
        midPointHeight = (frame.shape[1]//2, 0)
        midPointBase = (frame.shape[1]//2, frame.shape[0])
        if not set:
            followPoint = midPointBase
            set = True

        

        ############ Canny Edge Detection ###########
        blurred = cv.GaussianBlur(masked.copy(), (7,7), 0)
        edges = cv.Canny(masked.copy(), 127,255)

        maskedContours, _ = cv.findContours(edges.copy(), cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)
        maskedEdges = cv.drawContours(frameShown.copy(), maskedContours , -1, (0,0,255),1)
        cv.imshow('Masked Edges', maskedEdges)
        
        
        grayed = cv.cvtColor(frame.copy(), cv.COLOR_BGR2GRAY)
        blurredThresh = cv.GaussianBlur(grayed, (7,7), 0)
        _, thresh = cv.threshold(blurredThresh.copy(), 127,255, cv.THRESH_BINARY)
        threshEdges = cv.Canny(thresh, 127,255)
        threshContours, _ = cv.findContours(threshEdges, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        cv.drawContours(frame, threshContours, -1 , (255,134,255),thickness=3)
        cv.imshow('Thresh', frame)
        
        pointss = getShapeAndStuff(threshContours)
        # try:
        #     readBarcode( threshContours )
        # except:
        #     pass
        code = 3
        # reada barcode if follow point is over the barcode
        text = 'Not Folloing'
        if pointss[0] == 'Circle':
            SetMode(AUTOMATIC)
            followPoint = pointss[1]
            text = f'Following {followPoint}'

        elif pointss[0] == 'Rectangle':
            SetMode(AUTOMATIC)
            code = pointss[1]

            if code in greenCodes:
                mask = greenMask

            elif code in blueCodes:
                mask = blueMask

            else:
                mask = redMask
        
        else:
            text = 'Not Following'
            SetMode(MANUAL)



              
        if(followPoint != midPointBase):
            Input = followPoint[0]
            print(f'Input = {Input} Output = {Output}')
        Compute()
        
                               
            
        ######## Add the Line at Last To not affect Edge Detection #####   
        cv.line(frameShown, midPointBase, midPointHeight, (255,255,255), 2)
        cv.line(frameShown, midPointBase, followPoint, (231,93,165))

        cv.putText(frameShown, text, (0, 30), 4, 1, (225,225,250), 1)
        cv.imshow('Main', frameShown)
        

    if cv.waitKey(20) & 0xFF==ord('d'):
        if espOn:
            stopMoving()
        cv.destroyAllWindows()
        break

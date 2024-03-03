# https://www.educative.io/answers/shape-detection-opencv
# https://docs.opencv.org/3.4/d3/d05/tutorial_py_table_of_contents_contours.html

import cv2 as cv

videoCap = True


def getShapeAndStuff(cont) -> list:

    circlePoints = [] #list(tuple())

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
        if vertices == 3:
            shape = "Triangle"
        elif vertices == 4:
            (x, y, w, h) = cv.boundingRect(approx)
            ar = w / float(h)
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        elif vertices == 5:
            shape = "Pentagon"
        else:
            shape = "Circle"

        if(shape == 'Circle'):
            if perimeter > 500:
                cv.putText(contours, f'{cx}, {cy} {perimeter}', (cx,cy), 1,1,(255,0,0),1)
                circlePoints.append(tuple([cx,cy]))
        
        
        
        ######## Add the Line at Last To not affect Edge Detection #####   
        cv.line(contours,midPointBase, midPointHeight, (255,255,255), 2)


        
        # cv.imshow('Drawn Image', frame)
        # cv.imshow('Threshold', thresh) 
    
    circlePoints.sort(key= lambda x:x[1], reverse= True)
    return circlePoints.copy()



if videoCap == True:
    # vid = cv.VideoCapture('http://192.0.0.4:8080/video')
    minimumLength = 1000
    vid = cv.VideoCapture(0)
#shape[0] gives height shape[1] gives length
    vid.set

    while True:

        ret, frame = vid.read()
        if ret == True:
            frame = cv.resize(frame,(640,480), )
            
            #Line To Compute PID Against(To be Calibrated)
            midPointBase = (frame.shape[1]//2, 0)
            midPointHeight = (frame.shape[1]//2, frame.shape[0])


            ##### Canny Edge Detection #####
            grayed = cv.cvtColor(frame.copy(), cv.COLOR_BGR2GRAY)
            blurred = cv.GaussianBlur(grayed, (7,7), 0)
            edges = cv.Canny(blurred.copy(), 127,255)
            cv.imshow('Canny', edges)
            
            cont, _ = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            contours = cv.drawContours(frame.copy(), cont, -1, (0,0,255),1)


            followPoint = midPointBase
            pointss = getShapeAndStuff(cont)

            if len(pointss) != 0:
                followPoint = pointss[0]

            cv.putText(contours, f'Following {followPoint}', midPointHeight, 4, 1, (225,225,250), 1)
            cv.imshow('Contours', contours)
            

        if cv.waitKey(20) & 0xFF==ord('d'):
            cv.destroyAllWindows()
            break
        

##########################TEST CODE NO USE###############################
else:

    image = cv.imread('circle4.jpg')
    quarter = cv.resize(image, (640,640) )
    cv.imshow('Original', quarter)

    gray = cv.cvtColor(quarter, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (7,7), 0)
    cv.imshow('Gray', blurred)

    rethresh, thresh = cv.threshold(blurred,150,255,cv.THRESH_BINARY)[1]
    cv.imshow('threshold', thresh, )

    cont, hier = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(quarter, cont, -1, (0,255,0), 2)
    cv.imshow('Contours', thresh)
   
    drawn = quarter
    print(len(cont))
    for contt in cont:
        # print(contt)
        M = cv.moments(contt)
        if(M['m00'] != 0):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print(f'${cx} ${cy}')
            drawn = cv.circle( drawn, (cx,cy), 3, (0,255,0),  2 )

    cv.imshow('Centroids', drawn)


    if cv.waitKey(0) & 0xFF==ord('d'):
            cv.destroyAllWindows()

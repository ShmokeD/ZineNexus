# https://www.educative.io/answers/shape-detection-opencv
# https://docs.opencv.org/3.4/d3/d05/tutorial_py_table_of_contents_contours.html

import cv2 as cv
import time

videoCap = True

if videoCap == True:
    vid = cv.VideoCapture('http://192.0.0.4:8080/video')
    # vid = cv.VideoCapture(0)
#shape[0] gives height shape[1] gives length
    vid.set



    while True:

        ret, frame = vid.read()

        if ret == True:
            frame = cv.resize(frame,(640,480), )
            
            midPoint = frame.shape[0]//2

            grayed = cv.cvtColor(frame.copy(), cv.COLOR_BGR2GRAY)
            blurred = cv.GaussianBlur(grayed, (7,7), 0)
            edges = cv.Canny(frame.copy(), 127,255)
            
            cont, _ = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            cont2, _ = cv.findContours(blurred, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            contours = cv.drawContours(frame.copy(), cont, -1, (0,0,255),2)
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
                    cv.circle( contours, (cx,cy), 3, (0,255,255),  2 )

                #Polygon Approximation
                epsilon = 0.04 * cv.arcLength(contour, True)
                approx = cv.approxPolyDP(contour, epsilon, True)
                cv.drawContours(frame, [approx], 0, (0, 255, 0), 2)

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


                cv.putText(contours, shape, (cx,cy), 1,1,(255,0,0),1)
                            
                cv.imshow('Contours', contours)
                cv.imshow('Drawn Image', frame)
            
                

            



       
        if cv.waitKey(20) & 0xFF==ord('d'):
            cv.destroyAllWindows()
            break
        


else:

    image = cv.imread('circle4.jpg')
    quarter = cv.resize(image, (640,640) )
    cv.imshow('Original', quarter)
    gray = cv.cvtColor(quarter, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (7,7), 0)
    cv.imshow('Gray', blurred)
    thresh = cv.threshold(blurred,150,255,cv.THRESH_BINARY)[1]

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

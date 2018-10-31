import cv2 as cv
import numpy as np
import time

cap = cv.VideoCapture(0)  # 或传入0，使用摄像头

while (True):
    ##start =  time.process_time()
    # 读取一帧
    ret, frame = cap.read()
    frame1 = frame[50:240,180:460]
    #frame1 = frame[60:360,160:480]
    
    #huidu 
    gray = cv.cvtColor(frame1,cv.COLOR_BGR2GRAY)
   
    #二值化
    ret, binary = cv.threshold(gray, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    
    cv.namedWindow('frame', cv.WINDOW_NORMAL)
    cv.imshow('frame', frame)
    cv.imshow('binary', binary)
    
    
    
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
cv.destroyAllWindows()


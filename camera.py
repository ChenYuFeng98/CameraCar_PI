import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)  # 或传入0，使用摄像头

while (True):

    # 读取一帧
    ret, frame = cap.read()

    # 把 BGR 转为 HSV
    ###hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # HSV中黑色范围
    ##lower_black = np.array([0, 0, 0])
    ##upper_black = np.array([180, 255, 46])

    # 获得黑色区域的mask
    ###mask = cv.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 46]))
    
    #二值化
    #ret, binary = cv.threshold(mask, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    #print("threshold value:%s"%ret)
    
    #去噪
    #mask = cv.GaussianBlur(mask, (3, 3), 0)
    
    # 和原始图片进行and操作，获得黑色区域
    #res = cv.bitwise_and(frame, frame, mask=mask)
    
    ##xgrad = cv.Sobel(mask, cv.CV_16SC1, 1, 0)
    ##ygrad = cv.Sobel(mask, cv.CV_16SC1, 0, 1)
    ##edge_output = cv.Canny(xgrad, ygrad, 50, 150)

    cv.namedWindow('frame', cv.WINDOW_NORMAL)
    cv.imshow('frame', frame)
    #cv.namedWindow('mask', cv.WINDOW_NORMAL)
    #cv.imshow('mask', mask)
    ##cv.namedWindow('edge_output', cv.WINDOW_NORMAL)
    ##cv.imshow('edge_output', edge_output)
    #cv.namedWindow('res', cv.WINDOW_NORMAL)
    #cv.imshow('res', res)

    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()


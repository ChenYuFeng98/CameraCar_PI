import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
from tkinter import *

#设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BOARD)
IN3 = 36
IN4 = 38
ENB = 40
IN1 = 35
IN2 = 37
ENA = 31
ServoPin = 12

#忽略警告信息
GPIO.setwarnings(False)
#电机引脚初始化操作
def motor_init():
    global pwm_ENA
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    global pwm_ENB
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
#设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    
def servo_init():
    global pwm_servo
    GPIO.setup(ServoPin, GPIO.OUT,initial=GPIO.LOW)
    #设置pwm引脚和频率为50hz
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
    time.sleep(0.1)
#定义一个脉冲函数，用来模拟方式产生pwm值
#时基脉冲为20ms，该脉冲高电平部分在0.5-
#2.5ms控制0-180度
    
    
def getc(num):
    if num != 0:
        return int(num/abs(num))
    else:
        return 1
    
        
def up(speed):
    #print('G')       
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   
#启动PWM设置占空比为100（0--100）
    pwm_ENA.ChangeDutyCycle(speed-30) 
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   
#启动PWM设置占空比为100（0--100）
    pwm_ENB.ChangeDutyCycle(speed-30)           
    
def turn(angle):     
    pwm_servo.ChangeDutyCycle(2.5 + 10 * (80 + angle)/180)
    print(angle)
    #pwm_servo.ChangeDutyCycle(0)
    #time.sleep(0.02)
           
def stop():
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(0)
    
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(0)
   
    pwm_servo.ChangeDutyCycle(2.5 + 10 * 80/180)
    
           
          
    
motor_init()
time.sleep(0.1)
servo_init()
       
cap = cv.VideoCapture(0) 
 # HSV中黑色范围
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 38])
time.sleep(0.1)
sj = 0
while (True):
    #start =  time.process_time()
    if sj >= 0:      
        # 读取一帧
        ret, frame2 = cap.read()
        frame1 = cv.resize(frame2, (320,240), interpolation=cv.INTER_AREA) 
        frame = frame1[30:180,70:250]
        
        # 把 BGR 转为 HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # 获得黑色区域的mask
        mask = cv.inRange(hsv, lower_black, upper_black)
        #cv.imshow('mask', mask)
        
        for i in range(6):
            cv.line(mask,(0,30*i),(180,30*i),[0,0,0],1)
        #划纵线
        #for i in range(7):
         #   cv.line(mask,(30*i,0),(30*i,150),[0,0,0],1)
        #轮廓处理
        zone = [0,0,0,0,0,0]
        area = [0,0,0,0,0,0]
        x = [0,0,0,0,0,0]
        for i in range(1,6):
            zone[i] = mask[30*(i-1):30*i, 0:180]     
            image,contours,hierarchy = cv.findContours(zone[i], 3, 1)
            #print(len(contours))
            if len(contours) > 0:
                #img = cv.medianBlur(image,5) #进行中值滤波
                b = 0
                cnt = contours[b]   #选取其中的第一个轮廓,这幅图像只有两个轮廓
                M = cv.moments(cnt)
                if M["m00"] != 0:
                    cX=int(M["m10"]/M["m00"])   #计算质心
                    cY=int(M["m01"]/M["m00"])                            
                    x[i] = cX
                    #cv2.drawContours(zone[i],contours[b],-1,(120,120,120),1)
                    cv.circle(zone[i],(cX,cY),2,(0,92,92),-1)
                    area[i] = cv.contourArea(contours[b])+cv.arcLength(cnt,True)
                    #轮廓周长print(cv2.arcLength(cnt,True))
                    #print(int(area[i]))
        
                    if area[1] > 300:
                        if (abs(x[1]-90) > 10) and (abs(x[1]-90) <= 30):
                            up(90)      
                            turn(getc(x[1]-90)*20)
                        elif (abs(x[1]-90) > 30) and (abs(x[1]-90) <= 50):
                            up(90)      
                            turn(getc(x[1]-90)*30)
                        elif (abs(x[1]-90) > 50) and (abs(x[1]-90) <= 70):
                            up(90)      
                            turn(getc(x[1]-90)*40)
                        elif (abs(x[1]-90) > 70) and (abs(x[1]-90) <= 90):
                            up(90)      
                            turn(getc(x[1]-90)*45)
                        elif abs(x[5]-90) > 40:
                            up(90)      
                            #turn(getc(x[5]-90)*getc(x[1]-90)*-40)
                            turn(0)
                        elif abs(x[5]-90) > 22:
                            up(90)      
                            turn(getc(x[5]-90)*getc(x[1]-90)*-30)
                        elif abs(x[5]-90) > 13:
                            up(90)      
                            turn(getc(x[5]-90)*getc(x[1]-90)*-20)
                        else:
                            up(100)    
                            turn(0)
                            
                    elif area[2] > 300:
                        if (abs(x[2]-90) > 60) and (abs(x[2]-90) <= 90):
                            up(90)      
                            turn(getc(x[2]-90)*45)
                        elif (abs(x[2]-90) > 40) and (abs(x[2]-90) <= 60):
                            up(90)      
                            turn(getc(x[2]-90)*40)
                            
                        elif (abs(x[2]-90) > 15) and (abs(x[2]-90) <= 40):
                            up(90)      
                            turn(getc(x[2]-90)*30)
                        else:
                            stop()
                            
                    elif area[3] > 300:
                        if (abs(x[3]-90) > 50) and (abs(x[3]-90) <= 90):
                            up(90)      
                            turn(getc(x[3]-90)*40)
                        elif (abs(x[3]-90) < 12) and (abs(x[4]-90) < 12) and (abs(x[5]-90) < 12):
                            stop()
                        else:
                            stop()
                    elif area[4] > 500:
                        if (abs(x[4]-90) > 20) and (abs(x[4]-90) <= 90):
                            up(90)      
                            turn(getc(x[4]-90)*45)
                        elif (abs(x[4]-90) < 12) and (abs(x[5]-90) < 12):
                            stop()
                        else:
                            stop()
                    elif area[5] > 300:
                        if abs(x[5]-90) > 30:
                            up(90)      
                            turn(getc(x[5]-90)*45)
                        else:
                            stop()
                  
                    else:
                        stop()
        sj = 0
        cv.imshow('mask', mask)   
        k = cv.waitKey(2) & 0xFF
        if k == 27:
            break
    else:
        sj = sj + 1
   
    #end = time.process_time()
    #print('Running time: %.2f Seconds'%(end-start))
    #a += 1
    #if a == 20:
    #    break
    
   
   
    

pwm_servo.stop()
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()    
cv.destroyAllWindows()



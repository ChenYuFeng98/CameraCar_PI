import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
from threading import Timer
import math
GPIO.setmode(GPIO.BOARD) # 设置GPIO口为BOARD编码方式
IN3 = 36
IN4 = 38
ENB = 40
IN1 = 35
IN2 = 37
ENA = 31
ServoPin = 12
GPIO.setwarnings(False) # 忽略警告信息


def motor_init(): # 电机引脚初始化操作
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
    
def servo_init():  # 脉冲函数，模拟方式产生pwm值基脉冲20ms，高电平0.5-2.5ms控制0-180度
    global pwm_servo
    GPIO.setup(ServoPin, GPIO.OUT,initial=GPIO.LOW)
    #设置pwm引脚和频率为50hz
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)
    time.sleep(0.05)
   
    
    
def getc(num): # 判断正负零
    if num != 0:
        return int(num/abs(num))
    else:
        return 0
    
        
def up(speed,Dspeed):     
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
    pwm_ENA.ChangeDutyCycle(speed+Dspeed)  # left
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(speed-Dspeed)  # right       
    
def turn(angle):     
    pwm_servo.ChangeDutyCycle(2.5 + 10 * (81 + angle)/180)
    time.sleep(0.01)

           
def stopmotor():
    pwm_servo.ChangeDutyCycle(2.5 + 10 * 81/180)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(0)   
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(0)
    time.sleep(0.01)

def endif():
    frameend = frame1[180:220,80:240]
    hsvend = cv.cvtColor(frameend, cv.COLOR_BGR2HSV)
    # 获得黑色区域的mask
    maskend = cv.inRange(hsvend, lower_black, upper_black)
    '''cv.imshow('maskend', maskend)
    k = cv.waitKey(2) & 0xFF'''
    #if k == 27:
    #    break
    image,contoursend,hierarchyend = cv.findContours(maskend, 3, 1)
    if len(contoursend) > 0:
        cntend = contoursend[0]   #选取其中的第一个轮廓,这幅图像只有两个轮廓
        areaend = cv.contourArea(contoursend[0])+cv.arcLength(cntend,True)
        Mend = cv.moments(cntend)
        if Mend["m00"] != 0:
            cXend=int(Mend["m10"]/Mend["m00"])   #计算质心
            angle = getc(cXend - 100) * 50
            turn(angle)
        else:
            stopmotor()
    else:
        stopmotor()
        
    

motor_init() 
time.sleep(0.01)

servo_init()
time.sleep(0.01)

global e
e = [0,0,0] #误差初始化
global angle
global Kp
global Ki
global Kd
global Se
global Ion
Ion = 0
angle = 0
Se = 0
Kp = 0.70#0.8
Ki = 0.001
Kd = 1.4#1.25

################Speed
global es
es = [0,0,0]
global speed
global Dspeed
global Kps
global Kis
global Kds
global Ses
global Ions
Ions = 0
speed = 0
Dspeed = 0
Ses = 0
Kps = 0.46
Kis = 0.001
Kds = 1.15
#mt = MyTimer()
#mt.start(0.2,printt)
lower_black = np.array([0, 0, 0]) #  HSV中黑色范围
upper_black = np.array([180, 255, 38]) #  HSV中黑色范围
cap = cv.VideoCapture(0) 

def show():
    t = Timer(0.05,show)
    t.start()
    print(angle,speed,Dspeed)

#t = Timer(0.05,show)
#t.start()
while (True):
    # 读取一帧
    start = time.process_time()
    ret, frame2 = cap.read()
    frame1 = cv.resize(frame2, (320,240), interpolation=cv.INTER_AREA) 
    #frame = frame1[30:180,70:250]
    frame = frame1[85:145,60:260]#65..115
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) # 把 BGR 转为 HSV
    mask = cv.inRange(hsv, lower_black, upper_black)# 获得黑色区域的mask
    cv.line(mask,(0,0),(200,0),[0,0,0],1) #画横线
    cv.line(mask,(0,60),(200,60),[0,0,0],1) #画横线

    image,contours,hierarchy = cv.findContours(mask, 3, 1) #轮廓处理
    #print(len(contours))
    if len(contours) > 0:
        cnt = contours[0]   #选取其中的第一个轮廓,这幅图像只有两个轮廓
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cX=int(M["m10"]/M["m00"])   #计算质心
            ##cY=int(M["m01"]/M["m00"])                            
            #cv2.drawContours(zone[i],contours[b],-1,(120,120,120),1) # 轮廓线
            ##cv.circle(mask,(cX,cY),2,(0,92,92),-1)  #  描质心
            area = cv.contourArea(contours[0])+cv.arcLength(cnt,True)     
            if (area > 100 and area < 9000):
                ##e[2] = e[1]
                e[1] = e[0]
                e[0] = cX - 100
                
                if (abs(angle) < 40):# 抗积分饱和
                    Se += e[0]
                if (abs(e[0]) > 35): #  积分分离
                    Ion = 0
                else:
                    Ion = 1
                if (abs(e[0]) < 10): # dead zone
                    Up = 0
                else:
                    Up = Kp * e[0]
                Ui = Ki * Se * Ion / 2 #  梯形积分
                Ud = Kd * (e[0]-e[1])
                angle =  Up + Ui + Ud
         
                if (abs(angle) > 55):   #  最大角度限制
                    angle = getc(angle) * 55

                turn(angle)
               
                '''es[1] = es[0]
                es[0] = abs(angle)     
                if (speed < 42):# 抗积分饱和
                    Ses += es[0]
            
                if (es[0] > 30): #  积分分离
                    Ions = 0
                else:
                    Ions = 1
                Ups = Kps * es[0]
                Uis = Kis * Ses * Ions / 2 #  梯形积分
                Uds = Kds * (es[0]-es[1])
                speed = - Ups - Uis - Uds + 50
                Dspeed = 6 * math.tan(angle * 0.019) # half difference
                if (abs(Dspeed) > 5.5):
                    Dspeed = getc(Dspeed) * 5.5
                '''
                speed = 50 - 0.2 * abs(angle)
                Dspeed = getc(angle) * 5
                
                up(speed,Dspeed)
                
            else:
                endif()
        else:
            endif()  
    else:
        endif()
        
    
    #print(angle,speed,Dspeed)
    cv.imshow('mask', mask)
    k = cv.waitKey(2) & 0xFF
    if k == 27:
        break
       
    end =  time.process_time()
    print('Running time: %.2f Seconds'%(end-start))

pwm_servo.stop()
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()    
cv.destroyAllWindows()
mt.stop()



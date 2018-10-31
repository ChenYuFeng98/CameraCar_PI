import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
from threading import Timer

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
        return 0
    
        
def up(speed):
    #print('G')       
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   
#启动PWM设置占空比为100（0--100）
    pwm_ENA.ChangeDutyCycle(speed) 
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(speed+5)           
    
def turn(angle):     
    pwm_servo.ChangeDutyCycle(2.5 + 10 * (81 + angle)/180)
    #time.sleep(0.01)
    #print(angle)
           
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
        #print("stop")

class MyTimer:
    def __int__(self):
        self._timer = None
        self._tm= None
        self._fn = None
        
    def _do_func(self):
        if (self._fn):
            self._fn()
            self._do_start()
            
    def _do_start(self):
        self._timer = Timer(self._tm,self._do_func)
        self._timer.start()
        
    def start(self, tm, fn):
        self._fn = fn
        self._tm = tm
        self._do_start()
    
    def stop(self):
        try:
            self._timer.cancel()
        except:
            pass
        
def printt():
     print (e[0],"--",angle)
     
motor_init()
time.sleep(0.1)
servo_init()
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
cap = cv.VideoCapture(0) 
 # HSV中黑色范围
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 38])
time.sleep(0.01)
#mt = MyTimer()
#mt.start(0.2,printt)

Kp = 0.83 #0.81
Ki = 0.001
Kd = 1.3 # fangdao kaitou1.1

while (True):
    # 读取一帧
    start = time.process_time()
    ret, frame2 = cap.read()
    frame1 = cv.resize(frame2, (320,240), interpolation=cv.INTER_AREA) 
    #frame = frame1[30:180,70:250]
    frame = frame1[90:140,60:260]#65..115
    # 把 BGR 转为 HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # 获得黑色区域的mask
    mask = cv.inRange(hsv, lower_black, upper_black)
    #cv.imshow('mask', mask)
    #画横线
    cv.line(mask,(0,0),(200,0),[0,0,0],1)
    cv.line(mask,(0,50),(200,50),[0,0,0],1)
    #轮廓处理
    image,contours,hierarchy = cv.findContours(mask, 3, 1)
    #print(len(contours))
    if len(contours) > 0:
        cnt = contours[0]   #选取其中的第一个轮廓,这幅图像只有两个轮廓
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cX=int(M["m10"]/M["m00"])   #计算质心
            ##cY=int(M["m01"]/M["m00"])                            
            #cv2.drawContours(zone[i],contours[b],-1,(120,120,120),1)
            ##cv.circle(mask,(cX,cY),2,(0,92,92),-1)
            area = cv.contourArea(contours[0])+cv.arcLength(cnt,True)
            
            if (area > 100 and area < 8000):
                '''zengliangshi
                Up = Kp * (e[0]-e[1])
                Ui = Ki * e[0]
                Ud = Kd * (e[0]-e[1]-e[2])
                #angle += Kp * (e[0]-e[1]) + Ki * e[0] + Kd * (e[0]-e[1]-e[2])
                angle +=  Up + Ui + Ud'''
                
                e[2] = e[1]
                e[1] = e[0]
                e[0] = cX - 100
                
                if (abs(angle) < 50):# kang jifen baohe
                    Se += e[0]
                if (abs(e[0]) > 35): #jifen fenli
                    Ion = 0
                else:
                    Ion = 1
                
                Up = Kp * e[0]
                Ui = Ki * Se * Ion / 2 #tixing jifen
                Ud = Kd * (e[0]-e[1])
                angle =  Up + Ui + Ud

                
                if (abs(angle) > 50):   #Angle Max
                    angle = getc(angle) * 50
                    
                #print (e[0],"--",angle)
                up(45)
                turn(angle)
            else:
                endif()
        else:
            endif()  
    else:
        endif()
    
    #hsv2 = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)
    # 获得黑色区域的mask
    #mask2 = cv.inRange(hsv2, lower_black, upper_black)
    #cv.line(mask2,(180,120),(280,120),[0,0,0],1)
    #cv.line(mask2,(180,520),(280,520),[0,0,0],1)
    #cv.namedWindow('mask', cv.WINDOW_NORMAL)
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



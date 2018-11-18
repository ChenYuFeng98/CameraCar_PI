from tkinter import *
import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
import threading 

'''
Scale组件设置一个指定范围
只需要指定它的from和to两个选项即可，但由于from本身是python关键字
所以为了区分需要在后边紧跟一个下划线：from_
'''
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
motor_init()
time.sleep(0.1)
servo_init()

global e
e = [0,0,0] #误差初始化
global angle
global Se
global Ion
Ion = 0
angle = 0
Se = 0
lower_black = np.array([0, 0, 0]) #  HSV中黑色范围
upper_black = np.array([180, 255, 38]) #  HSV中黑色范围

import random
global auto
auto = 0    
    


 
class App:
    def __init__(self,master):
        
        
        frame1 = Frame(master,padx = 5,pady = 10)

        Radiobutton(frame1,text = "巡线模式",variable = r,value = 1,command = self.line).grid(row = 0,column = 0)
        Radiobutton(frame1,text = "操作模式",variable = r,value = 2,command = self.control).grid(row = 0,column = 1)  
        r.set(2)
        label11 = Label(frame1, text = "左轮PWM：")
        label11.grid(row = 1,column = 0, sticky = SE)
        s11 = Scale(frame1,from_=20,to=100,variable = ls,orient=HORIZONTAL,length = 130)  #orient=HORIZONTAL设置水平方向显示
        s11.grid(row = 1,column = 1)

        label12 = Label(frame1, text = "右轮PWM：")
        label12.grid(row = 2,column = 0, sticky = SE)
        s12 = Scale(frame1,from_=20,to=100,variable = rs,orient=HORIZONTAL,length = 130)  #orient=HORIZONTAL设置水平方向显示
        s12.grid(row = 2,column = 1)

        frame1.grid(row = 1,column = 1)



        frame2 = Frame(master,pady = 10,padx = 5)

        Btup = Button(frame2,text='前进',command = self.toward,width = 6).grid(row = 1,column = 1,padx = 5)
        Btdown = Button(frame2,text='后退',command = self.back,width = 6).grid(row = 3,column = 1,padx = 5)
        Btstop = Button(frame2,text='STOP',command = self.stop,width = 6).grid(row = 2,column = 1,padx = 5)
        Btleft = Button(frame2,text='左转',command = self.left,width = 6).grid(row = 2,column = 0,padx = 5,pady = 5)
        Btright = Button(frame2,text='右转',command = self.right,width = 6).grid(row = 2,column = 2,padx = 5,pady = 5)

        frame2.grid(row = 1,column = 2)

        frame3 = Frame(master,padx = 5,pady = 5)

        label31 = Label(frame3, text = "P：")
        label31.grid(row = 1,column = 0, sticky = SE)
        P = Entry(frame3,textvariable = Kp,width = 10)         
        P.grid(row = 1,column = 1)

        label32 = Label(frame3, text = "I：")
        label32.grid(row = 2,column = 0, sticky = SE)
        I = Entry(frame3,textvariable = Ki,width = 10) 
        I.grid(row = 2,column = 1)

        label33 = Label(frame3, text = "D：")
        label33.grid(row = 3,column = 0, sticky = SE)
        D = Entry(frame3,textvariable = Kd,width = 10) 
        D.grid(row = 3,column = 1)

        label34 = Label(frame3, text = "初始速度：")
        label34.grid(row = 1,column = 3, sticky = SE)
        initv = Entry(frame3,textvariable = initvv,width = 10) 
        initv.grid(row = 1,column = 4)

        label35 = Label(frame3, text = "减速系数：") #转弯减速系数
        label35.grid(row = 2,column = 3, sticky = SE)
        kv = Entry(frame3,textvariable = vv,width = 10) 
        kv.grid(row = 2,column = 4)
        #photo = PhotoImage(format="png",file=r"//home//pi//Desktop//PiCar//test.png",height = 200, width = 300)
        #picture = Label(frame3, image=photo,height = 80, width = 400)
        #picture.grid(row = 1,column = 2,rowspan = 4)


        frame3.grid(row = 2,column = 1,columnspan = 4)

    def toward(self):      
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENA.ChangeDutyCycle(ls.get())
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENB.ChangeDutyCycle(rs.get())             
        pwm_servo.ChangeDutyCycle(2.5 + 10 * 82/180)
        time.sleep(0.35)
        pwm_servo.ChangeDutyCycle(0)
        time.sleep(0.02)
            
    def back(self):
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENA.ChangeDutyCycle(ls.get())
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENB.ChangeDutyCycle(rs.get())
        
        pwm_servo.ChangeDutyCycle(2.5 + 10 * 82/180)
        time.sleep(0.15)
        #pwm_servo.ChangeDutyCycle(0)
        time.sleep(0.01)
        
        
    def left(self):
        GPIO.output(IN1, GPIO.HIGH) 
        GPIO.output(IN2, GPIO.LOW)         
    #启动PWM设置占空比为100（0--100）
        pwm_ENA.ChangeDutyCycle(ls.get())
        GPIO.output(IN3, GPIO.HIGH) 
        GPIO.output(IN4, GPIO.LOW)         
    #启动PWM设置占空比为100（0--100）
        pwm_ENB.ChangeDutyCycle(rs.get())       
        pwm_servo.ChangeDutyCycle(2.5 + 10 * (83-40)/180)
        time.sleep(0.35)
        pwm_servo.ChangeDutyCycle(0)
        time.sleep(0.02)

        
    def right(self):
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENA.ChangeDutyCycle(ls.get())
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENB.ChangeDutyCycle(rs.get())      
        pwm_servo.ChangeDutyCycle(2.5 + 10 * (83+40)/180)
        time.sleep(0.35)
        pwm_servo.ChangeDutyCycle(0)
        time.sleep(0.02)

        
    def stop(self):
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENA.ChangeDutyCycle(0)
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)   
    #启动PWM设置占空比为100（0--100）
        pwm_ENB.ChangeDutyCycle(0)
        
        pwm_servo.ChangeDutyCycle(2.5 + 10 * 82/180)
        time.sleep(0.35)
        pwm_servo.ChangeDutyCycle(0)
        time.sleep(0.02)

    ########################line def
    def getc(self,num): # 判断正负零
        if num != 0:
            return int(num/abs(num))
        else:
            return 0
        
            
    def up(self,speed,Dspeed):     

        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)   
        #启动PWM设置占空比为100（0--100）
        pwm_ENA.ChangeDutyCycle(speed+Dspeed)  # left
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)   
        pwm_ENB.ChangeDutyCycle(speed-Dspeed)  # right       
        
    def turn(self,angle):     

        pwm_servo.ChangeDutyCycle(2.5 + 10 * (81 + angle)/180)
        time.sleep(0.01)

               
    def stopmotor(self):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * 81/180)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.LOW)   
        pwm_ENA.ChangeDutyCycle(0)   
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)   
        pwm_ENB.ChangeDutyCycle(0)
        time.sleep(0.01)

    def endif(self):
        global frame1
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
                angle = self.getc(cXend - 100) * 50
                self.turn(angle)
                
                speed = initvv.get() - float(vv.get()) * abs(angle)
                Dspeed = self.getc(angle) * 5    
                self.up(speed,Dspeed)
            else:
                self.stopmotor()
        else:
            self.stopmotor()


    def show(self):
        ##print('\n''PWM读数:',ls.get(),random.randint(0,9))
        global angle
        global e
        global Se
        global Ion
        global frame1
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
                        Up = float(Kp.get()) * e[0]
                    Ui = float(Ki.get()) * Se * Ion / 2 #  梯形积分
                    Ud = float(Kd.get()) * (e[0]-e[1])
                    angle =  Up + Ui + Ud
             
                    if (abs(angle) > 55):   #  最大角度限制
                        angle = self.getc(angle) * 55

                    self.turn(angle)
                   
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
                        Dspeed = self.getc(Dspeed) * 5.5
                    '''
                    speed = initvv.get() - float(vv.get()) * abs(angle)
                    Dspeed = self.getc(angle) * 5
                    
                    self.up(speed,Dspeed)
                    
                else:
                    self.endif()
            else:
                self.endif()  
        else:
            self.endif()
            
        
        #print(angle,speed,Dspeed)
        #cv.imshow('mask', mask)
        #k = cv.waitKey(2) & 0xFF

        if(auto == 1):
            timer = threading.Timer(0.01,self.show)
            timer.start()
            
    def control(self):
        self.stop()
        cap.release()
        cv.destroyAllWindows()
        global auto
        auto = 0
        self.stop()
        
    def line(self):
        global auto
        auto = 1
        global cap
        cap = cv.VideoCapture(0) 
        timer = threading.Timer(0.02,self.show)
        timer.start()


root =Tk()
r = IntVar()
r.set(1) 
ls = IntVar() #StringVar是Tk库内部定义的字符串变量类型,部件变量绑定
ls.set(45)
rs = IntVar() #StringVar是Tk库内部定义的字符串变量类型,部件变量绑定
rs.set(45)
Kp = StringVar() #StringVar是Tk库内部定义的字符串变量类型,部件变量绑定
Kp.set(0.7)
Ki = StringVar() #StringVar是Tk库内部定义的字符串变量类型,部件变量绑定
Ki.set(0.001)
Kd = StringVar()
Kd.set(1.4)
initvv = IntVar()
initvv.set(50)
vv = StringVar()
vv.set(0.2)
root.wm_title('Car Control')
app = App(root)

root.mainloop()
pwm_servo.stop()
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()

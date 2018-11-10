
from tkinter import *
import cv2
import numpy as np


src = cv2.imread('binary.png')
'''
Scale组件设置一个指定范围
只需要指定它的from和to两个选项即可，但由于from本身是python关键字
所以为了区分需要在后边紧跟一个下划线：from_
'''


root =Tk()
r = IntVar()
r.set(1) 
v = StringVar() #StringVar是Tk库内部定义的字符串变量类型,部件变量绑定
v.set(45)
def show():
    print('\n''垂直位置读数:',v.get())

frame1 = Frame(root,padx = 5,pady = 10)

Radiobutton(frame1,text = "巡线模式",variable = r,value = 1,command = show).grid(row = 0,column = 0)
Radiobutton(frame1,text = "操作模式",variable = r,value = 2,command = show).grid(row = 0,column = 1)  

label11 = Label(frame1, text = "左轮PWM：")
label11.grid(row = 1,column = 0, sticky = SE)
s11 = Scale(frame1,from_=20,to=100,variable = v,orient=HORIZONTAL,length = 130)  #orient=HORIZONTAL设置水平方向显示
#s1.set(40)
s11.grid(row = 1,column = 1)

label12 = Label(frame1, text = "右轮PWM：")
label12.grid(row = 2,column = 0, sticky = SE)
s12 = Scale(frame1,from_=20,to=100,orient=HORIZONTAL,length = 130)  #orient=HORIZONTAL设置水平方向显示
s12.set(40)
s12.grid(row = 2,column = 1)

frame1.grid(row = 1,column = 1)



frame2 = Frame(root,pady = 10,padx = 5)

Btup = Button(frame2,text='前进',command = show,width = 6).grid(row = 1,column = 1,padx = 5)
Btdown = Button(frame2,text='后退',command = show,width = 6).grid(row = 3,column = 1,padx = 5)
Btleft = Button(frame2,text='左转',command = show,width = 6).grid(row = 2,column = 0,padx = 5,pady = 5)
Btright = Button(frame2,text='右转',command = show,width = 6).grid(row = 2,column = 2,padx = 5,pady = 5)

frame2.grid(row = 1,column = 2)

frame3 = Frame(root,padx = 5,pady = 5)

label31 = Label(frame3, text = "P：")
label31.grid(row = 1,column = 0, sticky = SE)
P = Entry(frame3,text = "0.85",width = 10) 
#s1.set(40)
P.grid(row = 1,column = 1)

label32 = Label(frame3, text = "I：")
label32.grid(row = 2,column = 0, sticky = SE)
I = Entry(frame3,text = "0.0015",width = 10) 
#s1.set(40)
I.grid(row = 2,column = 1)

label33 = Label(frame3, text = "D：")
label33.grid(row = 3,column = 0, sticky = SE)
D = Entry(frame3,text = "1.1",width = 10) 
#s1.set(40)
D.grid(row = 3,column = 1)

photo = PhotoImage(format="png",file=r"binary.png",height = 200, width = 300)
picture = Label(frame3, image=photo,height = 80, width = 400)
picture.grid(row = 1,column = 2,rowspan = 4)


frame3.grid(row = 2,column = 1,columnspan = 4)

root.title("Pack - Example")

root.mainloop()


 


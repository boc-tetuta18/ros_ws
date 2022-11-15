#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

from pickle import NONE
import cv2
import numpy as np
import RPi.GPIO as GPIO
import sys
from matplotlib import pyplot as plt
from numpy.random import *
import time
import rospy
from msg import Adder


#initialize PID parameter

goal = 340 #cener of capture_img
dt = 1
e = 0 #error
e1 = 0 #pre error
acc = 0 #accumulation
dif = 0 #deviation
Kp = 30/300  #50/300
Ki = 0.1
Kd = 0.1

timekey = 0

t= 100

count_time = 0

#graff
x_list = []
y_list = []

x_list.append(0)
y_list.append(0)

def tic():
    global start_time_tictoc
    start_time_tictoc = time.time()

def toc(tag="elapsed time"):
    if "start_time_tictoc" in globals():
        print("{}: {:.9f} [sec]".format(tag, time.time() - start_time_tictoc))
    else:
        print("tic has not been called")


def polygon_area(N, P):
    return abs(sum(P[i][0]*P[i-1][1] - P[i][1]*P[i-1][0] for i in range(N)))/2

def para_in():
    # 初期化宣言 : このソフトウェアは"para_in"という名前
    rospy.init_node('para_in', anonymous=True)

    # nodeの宣言 : publisherのインスタンスを作る
    # input_dataというtopicにAdder型のmessageを送るPublisherをつくった
    pub = rospy.Publisher('input_data', Adder, queue_size=100)

    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    sw_status = 0

    # Adder型のmessageのインスタンスを作る
    msg = Adder()

    # ctl +　Cで終了しない限りwhileループでpublishし続ける

    while not rospy.is_shutdown():

        msg.status_sw = sw_status

        # publishする関数
        pub.publish(msg)
        print (msg.status_sw)

        r.sleep()

duty = 85

#GPIO initial set
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

#GPIO SWitch set
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)



p1 = GPIO.PWM(27, 10000) #50Hz
p2 = GPIO.PWM(22, 10000) #50Hz
p3 = GPIO.PWM(23, 10000) #50Hz
p4 = GPIO.PWM(24, 10000) #50Hz

p1.start(0)
p2.start(0)
p3.start(0)
p4.start(0)

fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v') #/cv2.VideoWriter_fourcc('H', '2', '6', '4')
#capsetup
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, fourcc)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 680)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
size = (w,h)
fps = int(cap.get(cv2.CAP_PROP_FPS))
video = cv2.VideoWriter('video1.mp4',fourcc,fps,size)

# initialize the cv2 QRCode detector
detector = cv2.QRCodeDetector()
test_data = "qr/2/1"
firstlookID = -1
points = []
qr_s = 0

#print(dir(GPIO))
while True:
    if __name__ == '__main__': 
        try:
            #print("6:",GPIO.input(6))
            #print("7:",GPIO.input(7))
            #print("5:",GPIO.input(5))
            #print('timekey:',timekey)
            frame, img = cap.read()
            output_img = img.copy()
            video.write(img)
            #detect and decode to QR
            retval, decoded_info, points, staraight_qrcode = detector.detectAndDecodeMulti(img) #decoded_info:(('str'),('str'),...)
            data_list = [] 
            for i in decoded_info:
                #print(i)
                data_list.append(list(map(int, i.split()))) #decoded_info[i]'s str are splited to int type and list type then append to data_list
                
                data_listex = [30, 2, 1] #rosからsubしたstuck_robotのID
                #print(data_list) ##confirm QR cap
            for i, data in enumerate(data_list): #data youso toridasi,  data_list:(('int,int,int'),('int,int,int'),...),  data:('int,int,int')
                if data == []:
                    #print(data)
                    continue
                elif data[1] == data_listex[1]:
                    #print(points[i])
                    print(polygon_area(4,points[i]))
                    qr_s = polygon_area(4,points[i])
                    if firstlookID == -1: #when camera look ID first time, lookID is changed 
                        firstlookID = data[2]

                    elif firstlookID == data[2]: #when camera look ID first time, get center of QR's points
                        S1 =  ((points[i][3][0]-points[i][1][0])*(points[i][0][1]-points[i][1][1])-(points[i][3][1]-points[i][1][1])*(points[i][0][0]-points[i][1][0]))/2
                        S2 =  ((points[i][3][0]-points[i][1][0])*(points[i][1][1]-points[i][2][1])-(points[i][3][1]-points[i][1][1])*(points[i][1][0]-points[i][2][0]))/2    
                        C1_x = points[i][0][0] + (points[i][2][0]-points[i][0][0])*S1/(S1 + S2)
                        C1_y = points[i][0][1] + (points[i][2][1]-points[i][0][1])*S1/(S1 + S2)
                            #print('hhhhhhhhhhhhhhhhhh')
                            #print(C1_x,C1_y)

                            #Pcontrol
                            
                        e = goal - C1_x
                        acc = acc + e*i
                        dif = (e - e1) / i
                            #print(C1_x)

                        output = Kp * e
                        e1 = e
                        duty_out = abs(np.clip(output,-50,50))
                        duty_in = np.clip(duty_out,20,50)
                            #print(duty_out)
                        x_list.append(i)
                        y_list.append(output)
                        if qr_s < 14000 and timekey == 0:
                            if 0 < C1_x < 340:
                                p1.ChangeDutyCycle(duty_out)
                                p2.ChangeDutyCycle(0)
                                p3.ChangeDutyCycle(0)
                                p4.ChangeDutyCycle(0)


                            elif 340 <= C1_x < 680:
                                p1.ChangeDutyCycle(0)
                                p2.ChangeDutyCycle(0)
                                p3.ChangeDutyCycle(duty_out)
                                p4.ChangeDutyCycle(0)


                            time.sleep(0.1)

                            p1.ChangeDutyCycle(0)
                            p2.ChangeDutyCycle(0)
                            p3.ChangeDutyCycle(0)
                            p4.ChangeDutyCycle(0)

                            time.sleep(0.65)    

                        elif qr_s >= 14000 and timekey == 0:  #if QR_S comes 
                            if 0 < C1_x < 320:
                                p1.ChangeDutyCycle(duty_out)
                                p2.ChangeDutyCycle(0)
                                p3.ChangeDutyCycle(0)
                                p4.ChangeDutyCycle(0)
                                count_time = 0

                            elif 320 <= C1_x <= 360:
                                count_time = count_time + 1
                                p1.ChangeDutyCycle(0)
                                p2.ChangeDutyCycle(0)
                                p3.ChangeDutyCycle(0)
                                p4.ChangeDutyCycle(0)
                                print('timekey:',timekey,count_time)
                                if count_time >= 4:
                                    p1.ChangeDutyCycle(duty)
                                    p2.ChangeDutyCycle(0)
                                    p3.ChangeDutyCycle(duty)
                                    p4.ChangeDutyCycle(0)
                                    time.sleep(3.8)
                                    p1.ChangeDutyCycle(0)
                                    p2.ChangeDutyCycle(0)
                                    p3.ChangeDutyCycle(0)
                                    p4.ChangeDutyCycle(0)
                                    timekey = 1


                            elif 360 < C1_x < 680:
                                p1.ChangeDutyCycle(0)
                                p2.ChangeDutyCycle(0)
                                p3.ChangeDutyCycle(duty_out)
                                p4.ChangeDutyCycle(0)

                                count_time = 0


                            time.sleep(0.1)

                            p1.ChangeDutyCycle(0)
                            p2.ChangeDutyCycle(0)
                            p3.ChangeDutyCycle(0)
                            p4.ChangeDutyCycle(0)

                            time.sleep(0.65)    
                        #docking is success. Then, contoroler is finish!
                    if (GPIO.input(6) + GPIO.input(7) + GPIO.input(5)) >= 2:
                        p1.ChangeDutyCycle(0)
                        p2.ChangeDutyCycle(0)
                        p3.ChangeDutyCycle(0)
                        p4.ChangeDutyCycle(0)
                        print('finish')
                        sw_status = 1

            para_in()
                        
                        

            cv2.imshow("QRCODEscanner", output_img)
            #c = sys.stdin.read(1)    
            #if c == 'q':
                #break

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except rospy.ROSInterruptException: pass

cap.release()
video.release()
cv2.destroyAllWindows()


GPIO.cleanup(27)
GPIO.cleanup(22)
GPIO.cleanup(23)
GPIO.cleanup(24)
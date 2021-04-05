#!/usr/bin/cv RPi.GPIO
import cv2
import numpy as np
from time import sleep
from signal import pause
import serial
import os

os.environ['DISPLAY'] = ':0'

frameWidth = 640
frameHeight = 480

if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
	#ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
	ser.flush()

#s = [0,1]
cap = cv2.VideoCapture(0)
cap.set(3,frameWidth)
cap.set(4,frameHeight)

def empty(a):
	pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640,240)
cv2.namedWindow("Thresholds")
cv2.resizeWindow("Thresholds", 640,240)
cv2.createTrackbar("H_HIGH ","Parameters",237,255,empty)
cv2.createTrackbar("S_HIGH ","Parameters", 243,255,empty)
cv2.createTrackbar("V_HIGH ","Parameters", 222,255,empty)
cv2.createTrackbar("H_LOW ", "Parameters", 115, 255, empty)
cv2.createTrackbar("S_LOW ", "Parameters", 158,255,empty)
cv2.createTrackbar("V_LOW ","Parameters", 43, 255,empty)
#cv2.createTrackbar("T1: ","Thresholds")
#cv2.createTrackbar("T2: ","Thresholds")



#cv2.setMouseCallback(frame)


def getContours(img,imgContour):
	global cent, area, b
	contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > 2000:
			cv2.drawContours(imgContour,cnt,-1,(0,255,0),5)
			pari = cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,0.02 * pari, True)
			#print(len(approx))
			x,y,w,h = cv2.boundingRect(approx)
			cent = x+(w/2)
			cv2.rectangle(imgContour, (x,y),(x + w, y + h), (255,0,0),5)
			cv2.putText(imgContour, "Points: " + str(len(approx)),(x + w + 20, y + 20),cv2.FONT_HERSHEY_COMPLEX, .7, (0,0,255),2)
			cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,255),2)
			b = True
			#return b
while True:
	_,frame = cap.read()
	#print(len(frame))
	imgContours = frame.copy()

	threshold1 = cv2.getTrackbarPos("T1: ","Thresholds")
	threshold2 = cv2.getTrackbarPos("T2: ","Thresholds")

	lg = np.array([cv2.getTrackbarPos("H_LOW ","Parameters"),cv2.getTrackbarPos("S_LOW ","Parameters"),cv2.getTrackbarPos("V_LOW ","Parameters")])
	ug = np.array([cv2.getTrackbarPos("H_HIGH ","Parameters"),cv2.getTrackbarPos("S_HIGH ","Parameters"),cv2.getTrackbarPos("V_HIGH ","Parameters")])
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	imgBlur = cv2.GaussianBlur(hsv,(7,7),1)
	mask = cv2.inRange(imgBlur , lg,ug)
	#print(len(mask))
	#imgBlur = cv2.GaussianBlur(frame,(7,7),1)
	imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
	#hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
	kernel = np.ones((5,5))
	imgDil = cv2.dilate(mask, kernel, iterations = 1)
	b = False
	getContours(imgDil, imgContours)


#	cv2.imshow('frame',frame)
#	cv2.imshow('mask',mask)

	cv2.imshow("Frame: ",frame)
	cv2.imshow("Mask: ",mask)
	cv2.imshow("Contours: ", imgContours)
	print(b)
	#print(area)
	if (b == True and area < 100000):
		#print(cent)
		if (cent < 180):
			ser.write(str(2).encode('utf-8'))
			#sleep(.4)
			print("left")
		if (cent > 460):
			ser.write(str(3).encode('utf-8'))
			print("right")
			#sleep(.4)
		if (cent >=  180 and cent <= 460):
			ser.write(str(1).encode('utf-8'))
			print("straight")
			#sleep(.4)
	else:
		ser.write(str(0).encode('utf-8'))


	if ser.in_waiting >0:
		line = ser.readline().decode('utf-8').rstrip()
		print(line)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
			#print(x+(w/2))

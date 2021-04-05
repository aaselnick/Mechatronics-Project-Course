#!/usr/bin/cv RPi.GPIO
import cv2
import numpy as np
from time import sleep
from signal import pause
import serial
import os

#This line solves the issue of being unable to run the code in ssh given the
#error "Cannot connect to X server"
os.environ['DISPLAY'] = ':0'

# Frame size in pixels
frameWidth = 640
frameHeight = 480

ipt = []
#Establishes serial connection with the Arduino before the code can be run
if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
	#ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
	ser.flush()

# This commande activates the video capture
cap = cv2.VideoCapture(0)
cap.set(3,frameWidth)
cap.set(4,frameHeight)


def empty(a):
	pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640,240)
cv2.namedWindow("Thresholds")
cv2.resizeWindow("Thresholds", 640,240)

#Generates the slider bar window with values pre-set for the Red balloon
#as of now
cv2.createTrackbar("H_HIGH ","Parameters", 255,255,empty)
cv2.createTrackbar("S_HIGH ","Parameters", 233,255,empty)
cv2.createTrackbar("V_HIGH ","Parameters", 247,255,empty)
cv2.createTrackbar("H_LOW ", "Parameters", 103, 255, empty)
cv2.createTrackbar("S_LOW ", "Parameters", 126,255,empty)
cv2.createTrackbar("V_LOW ","Parameters", 119, 255,empty)
#cv2.createTrackbar("T1: ","Thresholds")
#cv2.createTrackbar("T2: ","Thresholds")



#cv2.setMouseCallback(frame)

#Function to draw contours based on the image and parameters provided
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

while ipt == []:
	ipt = input("Please Enter a Command: ")
	if ipt == "list":
		print("List of Commands:   ")
		print("Command:			What it Do:		Arduino Gets:")
		print("")
		print("F				  Forward	 	      70")
		print("B		    	   Back			      66")
		print("r     			  90deg CW 	         114")
		print("Q  			     180deg CW 			  82")
		print("L  				 FWD Line 			  76")
		ipt = []

	ser.write(char(ipt).encode('utf-8'))
# Main loop of the script
while ipt ~= [] and ipt ~= "list":
	_,frame = cap.read()
	#print(len(frame))
	imgContours = frame.copy()

	# Thresholds for contour stuff (unused as of now I think)
	threshold1 = cv2.getTrackbarPos("T1: ","Thresholds")
	threshold2 = cv2.getTrackbarPos("T2: ","Thresholds")

	# Higher and lower thresholds of the HSV values
	lg = np.array([cv2.getTrackbarPos("H_LOW ","Parameters"),cv2.getTrackbarPos("S_LOW ","Parameters"),cv2.getTrackbarPos("V_LOW ","Parameters")])
	ug = np.array([cv2.getTrackbarPos("H_HIGH ","Parameters"),cv2.getTrackbarPos("S_HIGH ","Parameters"),cv2.getTrackbarPos("V_HIGH ","Parameters")])
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

	# imgBlue is a blured version of the original hsv masked image such that
	# higher quality contours can be drawn around the ballons
	imgBlur = cv2.GaussianBlur(hsv,(7,7),1)

	# mask is the masked image of the blured hsv that seeks values of h,s,and v
	# within the specified bounds
	mask = cv2.inRange(imgBlur, lg, ug)

	#print(len(mask))

	#imgGray and Canny are unused currently, haven't figured out if this is
	#Something we would like to pursue in the future.
	imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
	imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
	kernel = np.ones((5,5))
	imgDil = cv2.dilate(mask, kernel, iterations = 1)
	b = False
	getContours(imgDil, imgContours)


#	Image outputs for the pure image, the masked image, and the contours

	cv2.imshow("Frame: ",frame)
	cv2.imshow("Mask: ",mask)
	cv2.imshow("Contours: ", imgContours)
	print(b)
	#print(area)
	if (b == True and area < 100000):
		#print(cent)
		if (cent < 80):
			ser.write(str(2).encode('utf-8'))
			#sleep(.4)
			print("left")
		if (cent > 510):
			ser.write(str(3).encode('utf-8'))
			print("right")
			#sleep(.4)
		if (cent >=  80 and cent <= 510):
			ser.write(str(1).encode('utf-8'))
			print("straight")
			#sleep(.4)
	else:
		ser.write(str(0).encode('utf-8'))

	# Command to read back what was sent to the Arduino
	if ser.in_waiting >0:
		line = ser.readline().decode('utf-8').rstrip()
		print(line)
	# If this if statement is removed, the output will be just a still Image
	# not a video
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
			#print(x+(w/2))

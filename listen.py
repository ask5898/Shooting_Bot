#!/usr/bin/env python

import sys
import pypot.dynamixel
import time
import itertools
import numpy as np
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from threading import Thread
from math import pi,atan,sin,cos,degrees
#import rospy
#from std_msgs.msg import String

#ang=(91.38, 87.34, 6.81, -47.16, 79.87, -80.31, -94.9, 124.18, -0.31, -2.68, 11.47, -12.7, -15.78, 14.55, -8.48, 3.91, -0.13, -4.26, 46.99)
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
hand = {5: 60, 6: -60}
port = input(sys.argv[0])
path = "/home/ali/catkin/src/walk/scripts/data.xml"


def sniper1():
	
	s=0
	
	cap = cv2.VideoCapture(1)
	y,u,v = 166,87,102
	while True:
		_,img = cap.read()
		img_yuv = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)
		copy = img.copy()
		blur = cv2.GaussianBlur(img_yuv,(7,7),2)


		die = cv2.inRange(blur, (np.array([y-45,u-30,v-30])), (np.array([y+45,u+30,v+30])))
		#cv2.imshow("The Masked Image",die)
		im_floodfill = die.copy()
	 	h, w = die.shape[:2]
		mask = np.zeros((h+2, w+2), np.uint8)
		abcd = cv2.floodFill(im_floodfill, mask, (0,0), 255);
		    
		fill = cv2.bitwise_and(im_floodfill,im_floodfill,mask = die)
		
		#cv2.imshow("Masked filled",fill)
	 	if cv2.waitKey(25)&0xff==27:
			break

		images,contours,hierarchy = cv2.findContours(fill,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

		cv2.drawContours(img, contours, -1, (255,0,255), 2)
		cv2.imshow("",img)
		print len(contours)

		s = s+len(contours)

		print s
		if s >= 40 :
			print "hi"
			cv2.imwrite("ball.jpg", copy)
			return True
			break
		else :
			return False

	cv2.destroyAllWindows()
	cap.release()

def sniper2():
	y,u,v = 166,87,102

	img =cv2.imread("ball.jpg")
	img_yuv = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)

	blur = cv2.GaussianBlur(img_yuv,(7,7),2)


	die = cv2.inRange(blur, (np.array([y-45,u-30,v-30])), (np.array([y+45,u+30,v+30])))
	#cv2.imshow("The Masked Image",die)
	im_floodfill = die.copy()
	h, w = die.shape[:2]
	mask = np.zeros((h+2, w+2), np.uint8)
	abcd = cv2.floodFill(im_floodfill, mask, (0,0), 255);
	    
	fill = cv2.bitwise_and(im_floodfill,im_floodfill,mask = die)

	#cv2.imshow("Masked filled",fill)


	images,contours,hierarchy = cv2.findContours(fill,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	#cv2.drawContours(img, contours, -1, (255,0,255), 2)
	#cv2.imshow("",img)
	print len(contours)


	cnt = contours[0]

	(x,y),radius = cv2.minEnclosingCircle(cnt)
	center = (int(x),int(y))
	radius = int(radius)
	point = cv2.circle(img,center,radius,(0,0,255),2)

	#cv2.imshow("abcd",point)
	cv2.imwrite("img.jpg",point)

	print center
	print radius


class Dynamixel(object) :
	def __init__(self) :
		ports = pypot.dynamixel.get_available_ports()
		if not ports :
			raise IOError("port bhakchodi pel rahe hain")

		print "Is port se judna hai",ports[0]

		self.dxl = pypot.dynamixel.DxlIO(ports[0])
		self.ids = self.dxl.scan(range(20))
		print self.ids
		self.dxl.enable_torque(self.ids)
		if len(self.ids)<19 :
			raise RuntimeError("kuch motor bhakchodi pel rahe hain")


	def setSpeed(self,speed,ids) :
		self.dxl.set_moving_speed(dict(zip(ids,itertools.repeat(speed))))

	def setPos(self,pose) :
		pos = {ids:angle for ids,angle in pose.items()}
		self.dxl.set_goal_position(pos)
		print pos

	def listWrite(self,list) :
		pos = dict(zip(self.ids,list))
		self.dxl.set_goal_position(pos)

	def dictWrite(self,dicti) :
		
		self.dxl.set_goal_position(dicti)

	def angleWrite(self,ids,pose) :
		self.dxl.set_goal_position({ids:pose})
		
	def returnPos(self,ids) :

		return self.dxl.get_present_position((ids,))	



class XML(object) :
	def __init__(self,file) :
		try :
			tree = ET.parse(file)
			self.root = tree.getroot()
		except :
			raise RuntimeError("File nahi mil rahi")

	def parse(self,motion) :
		find = "PageRoot/Page[@name='" +motion+ "']/steps/step"
		steps = [x for x in self.root.findall(find)]
		p_frame = str()
		p_pose = str()
		for step in steps :
			Walk(step.attrib['frame'],step.attrib['pose'],p_frame,p_pose)
			p_frame = step.attrib['frame']
			p_pose = step.attrib['pose']
			
	
xml = XML(path)
x=Dynamixel()

class Walk(object) :
	def __init__(self,frame,pose,p_frame,p_pose) :
		self.frame = int(frame)
		self.begin = {}
		self.end = {}
		if not(p_pose) :
			self.frame_diff = 10
			p_pose = pose
		else :
			self.frame_diff = self.frame-int(p_frame) 

		for ids,pos in enumerate(map(float,p_pose.split())) :
			self.end[ids+1]=pos	

		for ids,pos in enumerate(map(float,pose.split())) :
			self.begin[ids+1]=pos
		
		self.set(offsets=[darwin,hand])

	def Offset(self,offset) :
		
		for key in offset.keys() :
			if offset[key] == 'i' :
				self.begin[key] = -self.begin[key]
				self.end[key] = -self.end[key]
			else :
				self.begin[key] += offset[key]
				self.end[key] += offset[key]
		
		

	def set(self,offsets=[]) :
		for offset in offsets :
			self.Offset(offset)
		self.motion() 

	def motion(self) :
		print self.begin
		print self.end
		write=[]
		ids=[]
		f_d=abs(self.frame_diff/10)
		for key in self.end.keys() :
			#pose_diff=abs(self.end[key]-self.begin[key])
			linp=np.linspace(self.end[key],self.begin[key],f_d)
			write.append(linp)
			#write.append(self.begin[key])
			ids.append(key)	
		print "out"
		for pose in zip(*write) :
			print "in"
			x.setPos(dict(zip(ids,pose)))
			time.sleep(0.1)

	
class Head(object) :
	def __init__(self) :
	
		self.head=19
		self.turn_right=0
		self.turn_left=93
		self.centre_pos=41
		self.diff_left=abs(self.centre_pos-self.turn_left)
		self.diff_right=abs(self.centre_pos-self.turn_right)
		#x.setSpeed(10,fire_ids[-1:])
		self.detect = False
		#x.setSpeed(50,fire_ids[:-1])

	def head_right(self) :
		x.setSpeed(20,[self.head])
		#linp=np.linspace(self.centre_pos,self.turn_right,self.diff_right)
		
		pose={self.head:self.turn_right}
		print pose
	
		x.dictWrite(pose)

	def head_left(self) :
		x.setSpeed(20,[self.head])
		#linp=np.linspace(self.centre_pos,self.turn_left,self.diff_left)
		pose={self.head:self.turn_left}
		print pose
		x.dictWrite(pose)
		
	def to_centre(self) :
		x.setSpeed(20,[self.head])
		#linp=np.linspace(self.turn_left,self.centre_pos,self.diff_left)
		pose={self.head:self.centre_pos}
		print pose
		x.dictWrite(pose)

	
h=Head()

class Motion(object) :
	def __init__(self) : 
		
		
		self.fire_ids=[2,4,6]
		self.armangle=87.52
		self.t=1
		self.d=6
		self.shoot()
		

	def shoot(self) :
		head=list(x.returnPos(19))[0]
		print head
		alpha=float(atan(self.d+self.t*sin(head))/self.t*cos(head))
		write=[self.armangle,alpha,-30.02]
		print degrees(alpha)
		pos=dict(zip(self.fire_ids,write))
		x.dictWrite(pos)
		print pos



if __name__ == "__main__" :

	balance = xml.parse("152 Balance")
	raw_input("Proceed?")
	sniper_final = Thread(target = sniper1 , args = ())
	sniper_final.daemon = True
	sniper_final.start()
	
	while True :
		
		for pose in range(-90,90,10) :
			fire = Thread(target = x.angleWrite , args = (19,pose,))
			fire.daemon = True
			fire.start()
			
			time.sleep(0.1)
			if sniper1() :
				m = Motion()
				fire.sleep()

		for pose in range(90,-90,-10) :
			fire = Thread(target = x.angleWrite , args = (19,pose,))
			fire.start()
			time.sleep(0.1)
			if sniper1() :
				m = Motion()
				fire.sleep()
			

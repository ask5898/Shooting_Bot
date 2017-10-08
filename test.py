import pypot.dynamixel
import time
import itertools
import numpy as np
from math import pi,atan,sin,cos,degrees


#ang=(91.38, 87.34, 6.81, -47.16, 79.87, -80.31, -94.9, 124.18, -0.31, -2.68, 11.47, -12.7, -15.78, 14.55, -8.48, 3.91, -0.13, -4.26, 46.99)
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
hand = {5: 60, 6: -60}

class Dynamixel(object) :
	def __init__(self) :
		ports=pypot.dynamixel.get_available_ports()
		if not ports :
			raise IOError("No ports found")

		print "connecting to",ports[0]

		self.dxl=pypot.dynamixel.DxlIO(ports[0])
		self.ids=self.dxl.scan(range(20))
		print self.ids
		self.dxl.enable_torque(self.ids)
		if len(self.ids)<8 :
			raise RuntimeError("Not all the motors were detected")


	def setSpeed(self,speed,ids) :
		self.dxl.set_moving_speed(dict(zip(ids,itertools.repeat(speed))))

	def setPos(self,pose) :
		pos={ids:angle for ids,angle in pose.items()}
		self.dxl.set_goal_position(pos)
		print pos

	def listWrite(self,list) :
		pos=dict(zip(self.ids,list))
		self.dxl.set_goal_position(pos)

	def writePos(self,dicti) :
		
		self.dxl.set_goal_position(dicti)
		
	def returnPos(self,ids) :

		return self.dxl.get_present_position((ids,))	


x=Dynamixel()

class XML(object) :
	def __init__(self,file) :
		try :
			tree=ET.parse(file)
			self.root=tree.getroot()
		except :
			raise RuntimeError("File not found")

	def parse(self,motion) :
		find="PageRoot/Page[@name='" +motion+ "']/steps/step"
		steps=[x for x in self.root.findall(find)]
		p_frame=str()
		p_pose=str()
		for step in steps :
			Walk(step.attrib['frame'],step.attrib['pose'],p_frame,p_pose)
			p_frame=step.attrib['frame']
			p_pose=step.attrib['pose']
			
x=Dynamixel()

class Walk(object) :
	def __init__(self,frame,pose,p_frame,p_pose) :
		self.frame=int(frame)
		self.begin={}
		self.end={}
		if not(p_pose) :
			self.frame_diff=10
			p_pose=pose
		else :
			self.frame_diff=self.frame-int(p_frame) 

		for ids,pos in enumerate(map(float,p_pose.split())) :
			self.end[ids+1]=pos	

		for ids,pos in enumerate(map(float,pose.split())) :
			self.begin[ids+1]=pos
		
			
		#self.motion()
		self.set(offsets=[darwin])

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
			time.sleep(0.08)


class Head() :
	def __init__(self) :
		#self.fire_ids=fire_ids
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
	
		x.writePos(pose)

	def head_left(self) :
		x.setSpeed(20,[self.head])
		#linp=np.linspace(self.centre_pos,self.turn_left,self.diff_left)
		pose={self.head:self.turn_left}
		print pose
		x.writePos(pose)
		
	def to_centre(self) :
		x.setSpeed(20,[self.head])
		#linp=np.linspace(self.turn_left,self.centre_pos,self.diff_left)
		pose={self.head:self.centre_pos}
		print pose
		x.writePos(pose)

	
h=Head()

class Motion(object) :
	def __init__(self) : 
		
		
		self.fire_ids=[2,4,6]
		self.armangle=0
		self.t=1
		self.d=6
		self.shoot()
		

	def shoot(self) :
		head=list(x.returnPos(19))[0]
		print head
		alpha=float(atan(self.d+t*sin(head))/t*cos(head))
		write=[self.armangle,head,-22.5]
		print degrees(alpha)
		pos=dict(zip(self.fire_ids,write))
		x.writePos(pos)
		print pos






if __name__=="__main__" :
	balance = xml.parse("152 Balance")
	
	t0=time.time()
	while True :
		#t=time.time()
		#(t-t	
		t=time.time()
		print int(t-t0)
		if(int(t-t0) == 20) :
			m=Motion()
			break	
		h.head_right() 
		time.sleep(5)
		
		h.head_left()
		time.sleep(5)
		
			
		
	

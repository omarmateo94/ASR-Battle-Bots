#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unitysim.msg import BoundingBox3d
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
import threading
import math
import string
import time

class fire_game:
	def __init__(self):
		for i in sys.argv:
			print i +"\n"
		if len(sys.argv) > 1:
			self.topic = sys.argv[1]
		else:
			self.topic = "/mateo/cmd_vel"
		self.topic2 = "/mateo/cannon"
		self.laserLock = threading.Lock()
		
		# Variables for callbacks
		self.laser = None
		self.health = None
		self.player = None
		self.cannon = None
		self.canshoot = None
		self.hp = None

		# Variables for set_commands
		self.state = 0
		self.minDist = 2
		self.maxDist = 7
		self.q = None
		self.prev_q = None
		self.counter = 0

		# Variables for laser
		self.left = 0
		self.front = 0
		self.right = 0

		# Publisher & Subscribers
		self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)
		self.pub2 = rospy.Publisher(self.topic2,String ,queue_size=10)
		self.sub = rospy.Subscriber("/mateo/scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/mateo/healthfinder", BoundingBox3d, self.health_callback)
		rospy.Subscriber("/mateo/playerfinder", BoundingBox3d, self.player_callback)
		rospy.Subscriber("/mateo/q", Quaternion, self.q_callback)
		rospy.Subscriber("/mateo/canshoot", Int32, self.canshoot_callback)
		rospy.Subscriber("/mateo/hp", Int32, self.hp_callback)
		rospy.init_node('MateoFireGame')
		self.prevHp = self.hp
		self.rate = rospy.Rate(10)
		self.ranges = [] #just a list with no entries

	# Callbacks
	def laser_callback(self, msg1):
		self.laserLock.acquire()
		self.laser = msg1
		self.laserLock.release()
	
	def health_callback(self, msg2):
		self.health = msg2

	def player_callback(self, msg3):
		self.player = msg3

	def canshoot_callback(self, msg4):
		self.canshoot = msg4

	def hp_callback(self, msg5):
		self.hp = msg5

	def q_callback(self, msg6):
		self.q = msg6

	# Laser functions
	def getRangesDist(self, start, end, size):
		sumRanges=0
		for i in range(start, end, 1):
			sumRanges += self.ranges[self.angletoindex(i* math.pi / 180)]
		sumRanges /= size
		return sumRanges
	
	def indextoangle(self, x):
		if self.laser !=None:
			return x * self.laser.angle_increment + self.laser.angle_min

	def angletoindex(self, x):
		if self.laser != None:
			return int(math.floor((x - self.laser.angle_min) / self.laser.angle_increment))	

	#Wander function
	def wander(self):
		horSum = 0 #forward
		verSum = 0 #lateral
		for i in range(len(self.ranges)):
			horSum += math.cos(self.indextoangle(i)) * self.ranges[i] / self.laser.range_max 	
			verSum += math.sin(self.indextoangle(i)) * self.ranges[i] / self.laser.range_max 

		horSum /= len(self.ranges) 
		verSum /= len(self.ranges)
		#Checking for health and players

		theta =  math.atan2(verSum,horSum)
		summa = 0

		for i in range(-15, 15, 1):
			summa += self.ranges[self.angletoindex(i* math.pi / 180)] / self.laser.range_max
		summa /= 30
		if self.left < self.minDist or self.right < self.minDist or self.front < self.minDist:
			self.state = 1
			if self.left < self.right:
				self.move(-1, 1)
			else:
				self.move(-1, -1)

		twist = Twist()
		twist.linear.x = summa *4
		twist.angular.z = theta 
		self.pub.publish(twist)

	# Twist Functions
	def move(self, x, z):
		twist = Twist()
		twist.linear.x = x
		twist.angular.z = z
		self.pub.publish(twist)

	def turn_left(self):
		self.move(0.25, 0.25)

	def turn_right(self):
		self.move(0.25, -0.25)

	def move_forward(self):
		summa = 0
		for i in range(-10, 10, 1):
			summa += self.ranges[self.angletoindex(i* math.pi / 180)] / self.laser.range_max
		summa /= 5
		self.move(summa, 0)

	# Shoot Function
	def shoot(self):
		shoot = "Fire"	
		self.pub2.publish("Fire")
	
	# Main loop
	def main_loop(self):
		while not rospy.is_shutdown():
			self.laserLock.acquire()
			if self.laser != None:
				self.ranges = [] #makes the list of ranges empty again
				for i in range (len(self.laser.ranges)):
					if(math.isinf(self.laser.ranges[i])):
						self.ranges.append(self.laser.range_max)
					else:
						self.ranges.append(self.laser.ranges[i])
				#print(self.front)
				#self.wander()
				self.set_command()
			self.laserLock.release()
			self.rate.sleep()

	# Command Function
	def set_command(self):
		# Finding out how close the robot is from the wall
		self.left = self.getRangesDist(10,90,80)
		self.front = self.getRangesDist(-10,10,20)
		self.right = self.getRangesDist(-90,-10,80)
		total_laser = self.getRangesDist(-90,90,180) # This will help us to determine how close we are from the wall.
		
		print(self.left)
		if self.health != None:
			self.state = -2		
		
		elif self.player != None:
			self.state = -1
			theta = self.player.center.position.y * -1

		#State (-2): Find Health
		if self.state == -2:
			self.move(4, self.health.center.position.y * -1)
			if total_laser < 5:
				self.state = 1
				self.health = None
		
		# State (-1): Find Player
		elif self.state == -1:
			rospy.loginfo("Player Found")
			self.move(0.5, self.player.center.position.y * -1)
			self.shoot()
			self.player = None
			self.state = 0

		# State (0): wander()
		elif self.state == 0:
			self.wander()
	
		# State (1): backwards()
		elif self.state == 1:
			self.move(-1, 0)
			if total_laser > 10:
				self.state=0
		elif total_laser < 6:
			self.state = 1



if __name__ == '__main__':
	temp = fire_game()
	temp.main_loop()





















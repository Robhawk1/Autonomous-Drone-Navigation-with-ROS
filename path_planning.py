#!/usr/bin/env python

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):

		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]
		self.init_pos = [5.68,-1.91,33.40]
		self.pos = 0
		# [x_setpoint, y_setpoint, z_setpoint]
		self.test = 0
		self.set_pos = []
		self.setpoint = [5.68,-1.91,33.40]
		self.Iterm=[0.0,0.0,0.0]
		self.error=[0.0,0.0,0.0]
		self.prev_error=[0.0,0.0,0.0]
		self.max_values=[1200,1800]
		self.setpoint = self.init_pos
		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0
		self.Kp = [15.24,14.34,18.54]
		self.Ki = [0,0,0,0]
		self.Kd = [210,186.3,361]
		self.i = 0 			# It is the incrementing variable to switch path points
		self.g = 1			#It is to publish the requirement of next goal


		self.sample_time = 0.060 # in second


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.command_pub3 = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.command_pub1 = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.command_pub2 = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.command_pub4 = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.command_pub5 = rospy.Publisher('/zero_line', Int16, queue_size=1)
		self.new_path = rospy.Publisher('/new_path', Int16, queue_size = 1)
		self.change_goal = rospy.Publisher('/change_goal', Int16, queue_size = 1)


		#------------------------Add other ROS Publisher"s here-----------------------------------------------------







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/vrep/waypoints', PoseArray, self.whycon_input)





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		
		

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
	#Callback function to store pose array of the path waypoints 
	def whycon_input(self,msg):
		self.set_pos = []
		self.g += 1
		
		for i in range(60):
			a=[msg.poses[i].position.x,msg.poses[i].position.y,msg.poses[i].position.z]
			self.set_pos.append(a)
		
		

		#---------------------------------------------------------------------------------------------------------------

	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06 
		self.Ki[0] = alt.Ki * 0.008
		self.Kd[0] = alt.Kd * 0.3

	def roll_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.06 
		self.Ki[1] = alt.Ki * 0.008
		self.Kd[1] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------






	#Function to check error in the start of the simulation
	def check_pid1(self):
		self.pid()
		if(abs(self.error[0])<0.03 and abs(self.error[2])<0.03 and abs(self.error[2])<0.03):
			return 1
		else:
			return 0
	#Function to check error while traversing the waypoints
	def check_pid(self):
		
		self.pid()
		if(abs(self.error[0])<0.5 and abs(self.error[2])<0.5 and abs(self.error[2])<0.5):
			return 1
		else:
			return 0


	#Function to follow path
	def path_follow(self):
		self.pid()
		self.check_error()

		
	#Function to go from start to init_waypoint
	def init_path(self):
		self.pid()
		test = self.check_pid1()
		if(test == 1):
			self.new_path.publish(1)
			

		
	def check_error(self):
		
		if( len(self.set_pos) and self.setpoint == self.set_pos[-1]):	#Checking if the end of waypoints have been reached
			
				
			print('new_path_compute')
			self.change_goal.publish(self.g)
			self.new_path.publish(1)
			if(self.g == 4):
				self.disarm()
			self.i = 0
		elif(self.check_pid() and len(self.set_pos)):
			self.setpoint = self.set_pos[self.i]		#Updating the waypoints
			self.i += 1
			
			self.new_path.publish(0)
	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
		self.error[0] = self.drone_position[0] - self.setpoint[0]#pitch_error
		self.error[1] = self.drone_position[1] - self.setpoint[1]#roll_error
		self.error[2] = self.drone_position[2] - self.setpoint[2]#alt_error
		
		self.command_pub1.publish(self.error[0])
		self.command_pub2.publish(self.error[1])
		self.command_pub3.publish(self.error[2])
		
		self.command_pub5.publish(0)


		self.Iterm[0]=(self.Iterm[0]+self.error[0])*self.Ki[0]
		self.out_pitch=self.Kp[0]*self.error[0]+self.Iterm[0]+self.Kd[0]*(self.error[0]-self.prev_error[0])

		self.Iterm[1]=(self.Iterm[1]+self.error[1])*self.Ki[1]
		self.out_roll=self.Kp[1]*self.error[1]+self.Iterm[1]+self.Kd[1]*(self.error[1]-self.prev_error[1])

		self.Iterm[2]=(self.Iterm[2]+self.error[2])*self.Ki[2]
		self.out_alt=self.Kp[2]*self.error[2]+self.Iterm[2]+self.Kd[2]*(self.error[2]-self.prev_error[2])

		
		
		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1]
		self.prev_error[2] = self.error[2]
		

		self.cmd.rcPitch=1500+self.out_pitch
		self.cmd.rcRoll=1500+self.out_roll
		self.cmd.rcThrottle=1500+self.out_alt
		
		rospy.sleep(self.sample_time)

		if self.cmd.rcPitch > self.max_values[1]:
			self.cmd.rcPitch = self.max_values[1]

		if self.cmd.rcPitch < self.max_values[0]:
			self.cmd.rcPitch = self.max_values[0]

		if self.cmd.rcRoll > self.max_values[1]:
			self.cmd.rcRoll = self.max_values[1]

		if self.cmd.rcRoll < self.max_values[0]:
			self.cmd.rcRoll = self.max_values[0]

		if self.cmd.rcThrottle > self.max_values[1]:
			self.cmd.rcThrottle = self.max_values[1]

		if self.cmd.rcThrottle < self.max_values[0]:
			self.cmd.rcThrottle = self.max_values[0]



		self.command_pub.publish(self.cmd)




if __name__ == '__main__':

	e_drone = Edrone()
	j = 0
	while True:
		e_drone.init_path()
		
		if(e_drone.check_pid1() == 1):
			e_drone.change_goal.publish(1)
			e_drone.new_path.publish(1)
			
			break

	while not rospy.is_shutdown():
		
		e_drone.path_follow()
		

		
		

#! /usr/bin/env python
"""
.. module:: service_subscriber_Node_C
   :platform: Unix
   :synopsis: Python module responsible to subscribes the robot's position and velocity and retrieve the distance and velocity data  
.. moduleauthor:: Girum Molla mollagirum16@gmail.com

This node subscribes to a ROS topic */velxz_posxy*  that publishes messages with the robot's position and velocity and
an action topic */reaching_goal/goal* to obtain the targeted position that taken by the user. It then calculates the 
distance between the robot's position and a targeted position, as well as the average speed of the robot.
Then estabish the service *service_subscriber* 

Subscribes to:
   /velxz_posxy
   /reaching_goal/goal
      
""" 
import rospy
import math
import time
from assignment_2_2023.srv import Average, AverageResponse
from assignment_2_2023.msg import Vel_pos
from assignment_2_2023.msg import PlanningActionGoal


class Position_speed:
	def __init__(self):
		"""
        This is the constructor method of the *Position_speed* class.It initializes some variables and sets up a subscriber to listen to the */velxz_posxy* topic and */reaching_goal/goal*.
        """
		self.frequency = rospy.get_param('frequency')
		
		# Time of the last print
		self.old_time = 0
		# initialize the variables
		self.distance = rospy.get_param('des_pos_x')
		self.ave_vel = rospy.get_param('des_pos_y')
		self.targ_x=rospy.get_param('des_pos_x')
		self.targ_y=rospy.get_param('des_pos_y')
		# Create the service
		rospy.Service('service_position',Average,self.call_pos_vel)
		# Subscribes to the /velxz_posxy
		rospy.Subscriber('/velxz_posxy', Vel_pos, self.callback)
		# Subscribes to the /reaching_goal/goal
		rospy.Subscriber('/reaching_goal/goal',PlanningActionGoal,self.target)
  
	def target(self,msg):
		"""
		This function subscibe the /reaching_goal/goal obtain the message of the targeted position x and position y and then assign to a new variable.		
		"""
		self.targ_x=msg.goal.target_pose.pose.position.x 			# target position x
		self.targ_y=msg.goal.target_pose.pose.position.y			# target position y
	def callback(self,msg):
		"""
  		This callback function extracts the necessary information from the custom message and calculates the distance and average velocity. 
    	"""  
		# Get the current time in milliseconds
		current_time = time.time() * 1000
		# If the time difference is gretaer than the period store on the distance and ave_vel
		if current_time - self.old_time > 1000 / self.frequency:
			# Calculate the  distance from the desired position	to current position		
			self.distance = math.sqrt((self.targ_x - msg.pos_x)**2 + (self.targ_y - msg.pos_y)**2)
			# calculate the average velocity
			self.ave_vel = math.sqrt(msg.vel_x**2 + msg.vel_z**2)
			# update the time
			self.old_time = current_time 
	def call_pos_vel(self,req):
		"""
		This function response the distance between desired position and actual position and also return the average velocity of robot when call *service_subscriber* service.
    	"""
		return AverageResponse (self.distance, self.ave_vel)


def main ():
	rospy.init_node ('service_subscriber_Node_C')	# initialize the node
	Position_speed()								# run the class
	rospy.spin ()									# Await messages
	
	
if __name__ == '__main__':
	main()

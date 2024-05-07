#! /usr/bin/env python
"""
.. module:: service_return_Node_B
   :platform: Unix
   :synopsis: Python module responsible to returns the coordinates of the last target sent by the user
.. moduleauthor:: Girum Molla mollagirum16@gmail.com

This node implements a ROS service node. The code receives messages from an goal action,and responds returns the coordinates
of the last target sent by the user. The script creates a service that listens to the *reaching_goal/goal* topic and return the coordinates of the last target.

Subscribes to:
   /reaching_goal/goal

"""
# Import Libraries
import rospy
from assignment_2_2023.srv import Return, ReturnResponse
from assignment_2_2023.msg import PlanningActionGoal


class goal:
	def __init__(self):
		"""
		This function:
		
		- initializes the last targeted coordinate
		- creates the service
		- subscribes to the /reaching_goal/goal topic
		"""  
        
        # Initialize the x and y last targeted coordinate  
		self.targ_x = rospy.get_param('des_pos_x')
		self.targ_y = rospy.get_param('des_pos_y')
		# Create the service
		rospy.Service('service_goal',Return,self.callposition)
		# Subscribes to the /reaching_goal/goal
		rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.goal)
	def goal(self,msg):
		"""
		This function subscibe the /reaching_goal/goal obtain the message of the targeted position x and position y 
		and then assign to a new variable.
		"""
		self.targ_x=msg.goal.target_pose.pose.position.x
		self.targ_y=msg.goal.target_pose.pose.position.y
	def callposition(self,req):
		"""
		This function response the last target position x and position y when call *service_goal* service.
    	"""
		return ReturnResponse (self.targ_x, self.targ_y) # response targ_x and targ_y


def main ():
	rospy.init_node ('service_return_Node_B')   # initialize the node
	goal()  									# run the class
	rospy.spin ()								# Await messages
	
	
if __name__ == '__main__':
	main()

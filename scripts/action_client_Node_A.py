#!/usr/bin/env python

"""
.. module::action_client_Node_A
   :platform:Unix
   :synopsis:Python module which implements an action client that lets users set or cancel targets (x, y)

.. moduleauthor:: Girum Molla mollagirum16@gmail.com

This node manages user interaction to input coordinates (x, y) or cancel a target location for the robot.
It establishes a publisher (*pub*) responsible for broadcasting a custom message (*Velxz_posxy*) on the *velxz_posxy* topic.
The custom message encompasses four fields *msg_pos_x*, *msg_pos_y*, *msg_vel_x*, and *msg_vel_z* which convey the robot's position and velocity.

Subscribes to:
   /odom

Publishes to: 
   /velxz_posxy   
   
Action Client:
   /reaching_goal   

""" 
# Import Libraries
import rospy
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from std_srvs.srv import *
import sys
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2023.msg import Vel_pos
from colorama import Fore, Style
from colorama import init
init()

# intial publisher for all function (global variariable)
# publisher: sends a message which contains two parameters (position and velocity) 
pub = rospy.Publisher("/velxz_posxy", Vel_pos, queue_size = 10) 


# Subscriber's callback function
def call_back(msg):
    """
    Call_back function

	This function serves as a callback that triggers whenever a message is received from the *odom* topic.It extracts position and velocity details from the message and constructs a custom message incorporating these parameters.Subsequently, the function publishes the custom message to the *velxz_posxy* topic.
    
    """
 
    position = msg.pose.pose.position     # get the position information from the msg that are on /odom topic
    velocity = msg.twist.twist.linear     # get the velocity information from the msg that are on /odom topic
    velxz_posxy = Vel_pos()               # create custom message
    
    # set the custom message's parameters from /odom topic
    velxz_posxy.pos_x = position.x
    velxz_posxy.pos_y = position.y
    velxz_posxy.vel_x = velocity.x
    velxz_posxy.vel_z = velocity.z
    pub.publish(velxz_posxy)                  # publish the custom message on /velxz_posxy topic
 
    
def action_client():
    """
    action_client function

	This function handles the action client by initiating an instance of the *SimpleActionClient* class and waiting for the action server to initialize. Afterwards, it engages in a loop, prompting the user to provide the target position. If the user inputs *c*, the function cancels the existing goal. Alternatively, it converts the user's input to float data types, constructs a goal message, and dispatches it to the action server.
    
    """ 
    
    # create the action client
    action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    action_client.wait_for_server()           # wait for the server to be started
    status_goal = False

    while not rospy.is_shutdown():
        # Ready the computer keyboard inputs
        print(Fore.BLUE + "Please enter the desired input of target position or type c to cancel it ")
            
        x_input_position = input(Fore.GREEN + "Position X: ")
        
        y_input_position = input(Fore.GREEN + "Position Y: ")
        
 	# If user entered 'c' and after the robot is reaching the goal position, the user can cancel the goal position 
        if x_input_position == "c" or y_input_position == "c":      
            action_client.cancel_goal()      # cancel the goal
            status_goal = False
        else:
            # Convert the data type of the numbers from string to float
            x_float = float(x_input_position)
            y_float = float(y_input_position)
            
            # creat the goal to send to the server
            goal = assignment_2_2023.msg.PlanningGoal()
            goal.target_pose.pose.position.x = x_float
            goal.target_pose.pose.position.y = y_float
            action_client.send_goal(goal)                      # send the goal data to the action server
            status_goal = True

def main():
    """
    main function
    
    This function serves as the primary function of the script. It begins by initializing the ROS node,
    establishing a publishe	named *velxz_posxy*, and setting up a subscriber named *odom*. 
    Subsequently, it invokes the *action_client()* function.
    
    """
    
    rospy.init_node('action_client_Node_A')                              # initialize the node
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, call_back)  
    action_client()                                                       # finally, call the function client

if __name__ == '__main__':
    main()

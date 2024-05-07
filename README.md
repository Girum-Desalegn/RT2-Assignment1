Assignment_2 The two-wheelded mobile robot
======================================

This assignment involves the movement of a two-wheeled mobile robot within a predefined area. The robot is constrained by a blocking task, preventing it from moving until it reaches the specified target.
To accomplish this, the tasks need to be implemented using an action server.

A two-wheeled mobile robot navigates a three-dimensional space, avoiding obstacles to reach a designated position. The robot operates within the Gazebo simulation environment, facilitating user interaction.

To control the robot's movement in this environment, an action server is implemented, employing the bug0 algorithm.

Create a new package, in which you will develop three nodes:
- (a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the
feedback/status of the action server to know when the target has been reached. The node also publishesthe
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom;
- (b) A service node that, when called, returns the coordinates of the last target sent by the user.
- (c) Anotherservice node thatsubscribes to the robot’s position and velocity (using the custom message) and
implements a server to retrieve the distance of the robot from the target and the robot’s average speed.

Finally, Create a launch file to start the whole simulation.

The nodes in the package
------------------------
Nodes represent individual processes responsible for specific computations or tasks within the ROS ecosystem. 
These are essentially programs that publish or subscribe to topics or include functionality for a ROS service. 
Nodes are designed to be highly specialized, focusing on a single task. Communication between nodes is facilitated through the exchange of messages. 
In our package, there are six Python files, each serving as a distinct node.

1. **bug_as.py**: This action server node is essential for guiding the robot to its target location. It receives the desired position from the client to initiate the movement.
2. **go_to_point_service.py**: Implemented in Python, this ROS node incorporates a navigation algorithm responsible for directing the robot to a specified destination. It acts as a service node within the system.
3. **wall_follow_service.py**: This Python script defines a service node enabling the robot to navigate around obstacles. Using laser scan data, the robot calculates distances to walls on its left, right, and front, facilitating obstacle avoidance.
4. **action_client_Node_A.py**: This Python script manages user input for the robot's target coordinates (x, y) or cancellation. It establishes a publisher named "pub" responsible for broadcasting a custom message, "Velxz_posxy," on the topic "/velxz_posxy." The custom message encompasses four fields: "msg_pos_x," "msg_pos_y," "msg_vel_x," and "msg_vel_y," conveying information about the robot's position and velocity.

Pseudocode 
----------
    Start

    # Initialize the ROS node and publisher
    Create a ROS node named "action_client_Node_A"
    Create a publisher named "velxz_posxy" of type Vel_pos

    # Subscribe to the "/odom" topic
    Create a subscriber named "sub_from_Odom" to the "/odom" topic

    # Start the action client
    Create an action client named "action_client" of type PlanningAction
    Wait for the action server to start

    # loop
    While the ROS node is not shut down:
        # Get user input
        Prompt the user to enter a target position or "c" to cancel the goal
        Read the user input for x and y coordinates

        # Convert user input to floats
        Convert the x and y coordinates from strings to floats

        # Create a goal message
        Create a PlanningGoal message
        Set the x and y coordinates of the target position in the goal message

        # Send the goal message to the action server
        Send the goal message to the action client

        # Check if the goal is canceled
        If the user entered "c":
            Call the action_client.cancel_goal() method to cancel the goal
    # main function 
    decleare the main function
    # Spin the node
    Spin the node to process incoming messages
    End

5. **Service_return_Node_B**: This Python script creates a service node that, when called, provides the coordinates of the last target sent by the user.
6. **Service_subscriber_Node_C**: This script represents another service node, which subscribes to the robot’s position and velocity through a custom message. It implements a server functionality to fetch the distance between the robot and the target, as well as the robot’s average speed.

launch file
-----------
A ROS launch file simplifies the startup of various components in your ROS system. It allows you to define parameters, start nodes, group them with prefixes, and use conditional statements. You can organize multiple launch files within one, enabling quick deployment for different robot startup modes. The roslaunch tool facilitates the effortless launch of multiple ROS nodes, locally adjusting parameters on the Parameter Server and automatically respawning processes if needed.
Here is a roslaunch file >*assignment2.launch*

    <?xml version="1.0"?>
    <launch>
        <include file="$(find assignment_2_2023)/launch/sim_w1.launch" />
        <param name="des_pos_x" value= "0.0" />
        <param name="des_pos_y" value= "1.0" />
        <param name="frequency" type="double" value="1.0" />
        
        <node pkg="assignment_2_2023" type="wall_follow_service.py" name="wall_follower" />
        <node pkg="assignment_2_2023" type="go_to_point_service.py" name="go_to_point"  />
        <node pkg="assignment_2_2023" type="bug_as.py" name="bug_action_service" output="screen" />
        <node pkg="assignment_2_2023" type="action_client_Node_A.py" name="action_client_Node_A" output="screen" launch-prefix="xterm -hold -e" />
        <node pkg="assignment_2_2023" type="service_return_Node_B.py" name="service_return_Node_B" />
        <node pkg="assignment_2_2023" type="service_subscriber_Node_C.py" name="service_subscriber_Node_C" />
        
    </launch>
Installation and Running Procedure
----------------------------------
Clone the package repository in the src/assignment_2_2023

```bash
git clone https://github.com/Girum-Desalegn/RT1-Assignment2.git
```
Install the xterm library
```bash
sudo apt-get install xterm -y
```
Build the workspace in your root folder

```bash
catkin_make 
```
Launch the simulation and all the required nodes. 
```bash
source /root/Assignment_2/devel/setup.bash
roslaunch assignment_2_2023 assignment2.launch
```
Further improvement
-------------------
The node developed in this assignment solely takes the desired target position of the moving robot. However, it would be more beneficial if the node incorporated the desired velocity, as this would facilitate smoother and more accurate movement. Moreover, employing an algorithm other than the bug 0 algorithm is recommended for enhanced robot movement precision.

Conclusion
----------
We have developed a package that contains three distinct nodes, each responsible for a specific task. The first node implements an action client, allowing the user to set or cancel a target. Based on these inputs, it also publishes the robot's position and velocity as a custom message on a designated topic. The second node is a service node that, when invoked, returns the coordinates of the last target specified by the user. The third service node, upon subscribing to the robot's position and velocity (custom message), calculates and reports the robot's average speed and distance from the target. To facilitate the launch of the entire simulation environment and our six different nodes, we also created a launch file.


#  Manipulate with Turtlesim Package in ROS 2

## 1. Introduction
The purpose of this task is to demonstrate practical manipulation of the turtlesim package in ROS 2.  
Turtlesim is a lightweight simulator used primarily for teaching core concepts of ROS (Robot Operating System), including topics, services, publishers, and subscribers. This report elaborates on how to control a turtle's movement, draw patterns, spawn additional turtles, and utilize ROS communication methods through a series of structured commands and scripts.

## 2. Objective
The primary objective is to use the turtlesim node to simulate turtle movements and control them using ROS 2 mechanisms such as:
- Velocity commands through topics
- Position control through services
- Spawning and managing multiple turtles
- Drawing defined patterns and shapes

This exercise lays foundational understanding for robot control and motion planning in future complex systems.

## 3. ROS 2 Concepts Used
- **Node**: An executable representing a process. E.g., `turtlesim_node` is responsible for displaying the turtle simulation window.
- **Topic**: A named communication channel over which nodes exchange messages. `/turtle1/cmd_vel` is used to move the turtle.
- **Publisher**: A node that sends messages to a topic.
- **Subscriber**: A node that receives messages from a topic.
- **Service**: Allows a node to send a request and receive a response. Services like `teleport_absolute`, `clear`, and `spawn` are utilized.

## 4. Command Execution and Steps

### Step 1: Launch the Turtlesim Node
```bash
ros2 run turtlesim turtlesim_node
```

### Step 2: Move the turtle by publishing to the cmd_vel topic
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

### Step 3: Use teleport service to move turtle instantly
```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.0, y: 5.0, theta: 0.0}"
```

### Step 4: Clear the canvas
```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

### Step 5: Spawn a second turtle
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 3.0, theta: 0.0, name: 'turtle2'}"
```

### Step 6: Move the second turtle
```bash
ros2 topic pub /turtle2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}"
```

## 5. Visual Results and Analysis

**Figure 1**: This image demonstrates a single turtle drawing a V-shape pattern using manually published velocity commands.  
The turtle follows a path influenced by a combination of linear and angular velocities.  
![Figure 1](https://github.com/user-attachments/assets/1e834c97-c562-4cc4-87c3-996f9c636a75)

**Figure 2**: Here, two turtles are used simultaneously. The first turtle performs angular turns while the second draws straight lines.  
This setup illustrates the power of multi-turtle spawning and coordinated motion using ROS 2 topics.  
![Figure 2](https://github.com/user-attachments/assets/c2c019f6-e256-485d-b33e-0e6b6a31604c)

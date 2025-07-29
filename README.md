# Task 3: Manipulate with Turtlesim Package in ROS 2

## 🐢 Overview
This task demonstrates how to manipulate turtles using the Turtlesim package in ROS 2. 
We'll control turtle movement using velocity commands, teleportation services, and spawn multiple turtles to draw shapes or patterns.

## 🛠️ Prerequisites
- ROS 2 installed (e.g., Humble or Foxy)
- `turtlesim` package available (`sudo apt install ros-<distro>-turtlesim`)

## 🚀 Launch Turtlesim
```bash
ros2 run turtlesim turtlesim_node
```

## 🎮 Control Turtle with Keyboard (Optional)
```bash
ros2 run turtlesim turtle_teleop_key
```

## 📦 Move Turtle Using Topic Commands
You can move the turtle using the `/turtle1/cmd_vel` topic by publishing velocity commands.
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```
This makes the turtle move forward and rotate.

## 🔁 Useful Services

- **Teleport to a position**:
```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.0, y: 5.0, theta: 0.0}"
```

- **Clear drawing**:
```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

- **Reset simulation**:
```bash
ros2 service call /reset std_srvs/srv/Empty "{}"
```

## 🧱 Spawn Additional Turtles
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 3.0, theta: 0.0, name: 'turtle2'}"
```
Use `/turtle2/cmd_vel` to move the second turtle.

## 🐍 Optional Python Script: Draw Triangle
You can create a script to automate turtle movement.
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TriangleDrawer(Node):
    def __init__(self):
        super().__init__('triangle_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        time.sleep(2)
        self.draw_triangle()

    def draw_triangle(self):
        twist = Twist()
        for _ in range(3):
            twist.linear.x = 2.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            time.sleep(2)

            twist.linear.x = 0.0
            twist.angular.z = 2.1
            self.pub.publish(twist)
            time.sleep(1.5)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = TriangleDrawer()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run the script using:
```bash
ros2 run <your_package> <your_script>
```

## 🧾 Summary
- You learned how to move and control turtles using topics and services.
- Multiple turtles were used to create collaborative patterns.
- Optional Python nodes allow automation of drawing.

---
This task is a foundational part of learning ROS 2 communication between nodes and topics.

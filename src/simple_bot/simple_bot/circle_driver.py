"""circle_driver.py - makes the robot drive around a circle.

THE IDEA
--------
A robot that moves forward at speed v (metres/second) while also turning at rate w
(radians/second) travels on a circle of radius R = v / w.
We want R = 5 m, so we pick a forward speed v and calculate w = v / R.

HOW A ROS 2 PROGRAM WORKS (very short version)
----------------------------------------------
* A program is called a NODE.
* Nodes talk by publishing messages on named TOPICS (like radio channels).
* This node publishes "Twist" messages (a speed + a turn rate) on the topic /cmd_vel.
  The simulated robot listens to /cmd_vel and moves its wheels accordingly.

LIMITATION
----------
This is "open loop": we send commands but never check where the robot really is.
If the wheels slip, the real circle will be a bit different. Later we can read /odom
(the robot's position estimate) and correct the path.
"""

# rclpy is the Python library for ROS 2.
import rclpy
# Twist is the message type for velocity commands: linear (forward) + angular (turn) speed.
from geometry_msgs.msg import Twist
# Node is the base class every ROS 2 node inherits from.
from rclpy.node import Node


class CircleDriver(Node):
    """A node that publishes the same velocity command over and over."""

    def __init__(self):
        # Give the node a name. It shows up in `ros2 node list`.
        super().__init__('circle_driver')

        # PARAMETERS: settings we can change without editing the code
        # (from the launch file or the command line). The second value is the default.
        self.declare_parameter('radius', 5.0)          # circle radius, metres
        self.declare_parameter('linear_speed', 0.5)    # forward speed, metres/second
        self.declare_parameter('publish_rate', 20.0)   # how many commands per second

        # Read the current parameter values.
        radius = self.get_parameter('radius').value
        self.linear_speed = self.get_parameter('linear_speed').value
        rate = self.get_parameter('publish_rate').value

        # A circle needs a positive radius. Stop early with a clear error otherwise.
        if radius <= 0.0:
            raise ValueError('radius must be greater than 0')

        # THE KEY FORMULA: turn rate w = v / R.
        # Positive value = turn left (counter-clockwise when seen from above).
        self.angular_speed = self.linear_speed / radius

        # PUBLISHER: the "mouth" of this node. It sends Twist messages to /cmd_vel.
        # The number 10 is the queue size (how many unsent messages may wait).
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # TIMER: calls self.on_timer() repeatedly, `rate` times per second.
        # The robot stops if it stops receiving commands, so we keep sending them.
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        # Print a summary to the terminal (circumference / speed = time for one lap).
        lap_time = 2.0 * 3.141592653589793 * radius / self.linear_speed
        self.get_logger().info(
            f'Driving a circle: radius={radius} m, v={self.linear_speed} m/s, '
            f'w={self.angular_speed:.3f} rad/s, one lap about {lap_time:.0f} s')

    def on_timer(self):
        """Runs many times per second: build one velocity command and send it."""
        msg = Twist()
        msg.linear.x = self.linear_speed     # forward speed along the robot's X axis
        msg.angular.z = self.angular_speed   # rotation around the vertical Z axis
        self.publisher.publish(msg)

    def stop_robot(self):
        """Send an all-zero command so the robot stops when we shut down."""
        try:
            self.publisher.publish(Twist())  # a Twist with all zeros means "stand still"
        except Exception:
            # ROS may already be shutting down; there is nothing more we can do.
            pass


def main(args=None):
    """Entry point. ROS starts the program by calling this function."""
    rclpy.init(args=args)          # 1. start ROS 2 communication
    node = CircleDriver()          # 2. create our node
    try:
        rclpy.spin(node)           # 3. keep running (timers fire) until Ctrl+C
    except KeyboardInterrupt:
        pass                       # Ctrl+C is the normal way to stop, not an error
    finally:
        node.stop_robot()          # 4. make the robot stand still
        node.destroy_node()        # 5. clean up
        if rclpy.ok():
            rclpy.shutdown()


# Lets you also run the file directly with `python3 circle_driver.py`.
if __name__ == '__main__':
    main()

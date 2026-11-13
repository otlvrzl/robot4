"""obstacle_avoider.py - drives the robot around a circle and steers around obstacles.

THE IDEA
--------
Like circle_driver.py, this node makes the robot drive a circle (radius 4 m). The
difference: it LISTENS to the robot's sensors, so it can react to what happens.

  * /scan (LIDAR)  -> tells us how far away things are in every direction.
  * /odom (odometry) -> tells us where the robot is (x, y) and where it points (yaw).
  * /cmd_vel       -> where we send our speed commands (same topic as circle_driver).

A STATE MACHINE (the heart of this file)
----------------------------------------
A state machine is just "the robot is always in exactly one mode, and simple rules
tell it when to switch to another mode". We have three modes:

  FOLLOW_CIRCLE --(obstacle closer than avoid_distance in front)--> AVOID
  AVOID         --(the way ahead is free again)-------------------> BYPASS
  BYPASS        --(obstacle is beside/behind us, side is clear)---> FOLLOW_CIRCLE

  FOLLOW_CIRCLE : drive the circle and correct the position using odometry, so the
                  robot also finds its way BACK to the circle after a detour.
  AVOID         : slow down and turn towards the side that has more free space.
  BYPASS        : drive forward past the obstacle until it is no longer beside us.

IMPORTANT ASSUMPTION
--------------------
The robot is spawned at (0, 0) looking along +X, and odometry starts at (0, 0) too.
A counter-clockwise circle of radius R from that start has its CENTRE at (0, R).
If you change the spawn position in sim.launch.py, change the centre here as well.
"""

import math

# rclpy is the Python library for ROS 2.
import rclpy
# Twist = velocity command (forward speed + turn rate).
from geometry_msgs.msg import Twist
# Odometry = the robot's estimated position and orientation.
from nav_msgs.msg import Odometry
from rclpy.node import Node
# Sensor data may be lost now and then; this QoS profile ("best effort") is the
# normal choice for sensors: newest data matters, old data does not.
from rclpy.qos import qos_profile_sensor_data
# LaserScan = one LIDAR measurement: a list of distances, one per laser ray.
from sensor_msgs.msg import LaserScan

# Names of the three states. Plain text is easy to read in the log.
FOLLOW_CIRCLE = 'FOLLOW_CIRCLE'
AVOID = 'AVOID'
BYPASS = 'BYPASS'


# --------------------------------------------------------------------------
# Small helper functions (pure maths, no ROS). Keeping them separate makes the
# node itself easier to read.
# --------------------------------------------------------------------------
def wrap_angle(angle):
    """Convert any angle to the range -pi ... +pi.

    Why: the difference between 359 deg and 1 deg is 2 deg, not 358 deg.
    Without wrapping, the robot would turn the long way round.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """Keep `value` between `low` and `high`."""
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    """Extract the heading (rotation around the vertical axis) from a quaternion.

    ROS stores orientation as a quaternion (x, y, z, w) - hard to read for humans.
    For a robot driving on flat ground only the heading angle "yaw" matters.
    yaw = 0 means looking along +X; positive yaw = turned left (counter-clockwise).
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sector_min_range(scan, low_angle, high_angle):
    """Smallest distance the LIDAR measured between two angles (radians).

    Angles are relative to the robot: 0 = straight ahead, + = left, - = right.
    Returns scan.range_max ("nothing seen") if the sector is empty.
    """
    closest = scan.range_max
    angle = scan.angle_min            # angle of the first laser ray
    for distance in scan.ranges:
        if low_angle <= angle <= high_angle:
            if math.isnan(distance):
                pass                  # broken measurement: ignore it
            elif distance < scan.range_min:
                # Gazebo reports "-inf" for things closer than range_min. That means
                # "something is VERY close", so we treat it as the smallest value.
                closest = min(closest, scan.range_min)
            else:
                # Note: "inf" (= nothing hit within range) is never smaller than
                # range_max, so it correctly changes nothing.
                closest = min(closest, distance)
        angle += scan.angle_increment  # move on to the next laser ray
    return closest


class ObstacleAvoider(Node):
    """Drives the circle, avoids obstacles seen by the LIDAR, returns to the circle."""

    def __init__(self):
        super().__init__('obstacle_avoider')

        # ---- PARAMETERS: settings you can change without editing the code ----
        # Circle settings (same meaning as in circle_driver.py).
        self.declare_parameter('radius', 4.0)           # circle radius, metres
        self.declare_parameter('linear_speed', 0.5)     # normal forward speed, m/s
        # Avoidance settings.
        self.declare_parameter('avoid_distance', 1.0)   # react to obstacles closer than this, m
        self.declare_parameter('avoid_turn_rate', 0.8)  # how fast to turn when avoiding, rad/s
        self.declare_parameter('avoid_speed', 0.15)     # forward speed while turning away, m/s
        self.declare_parameter('bypass_speed', 0.3)     # forward speed while passing the obstacle, m/s
        # Circle-following settings (how strongly to correct position errors).
        self.declare_parameter('radius_gain', 0.8)      # bigger = returns to the circle more eagerly
        self.declare_parameter('heading_gain', 1.5)     # bigger = follows the wanted heading more tightly
        self.declare_parameter('control_rate', 20.0)    # control decisions per second

        self.radius = self.get_parameter('radius').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.avoid_distance = self.get_parameter('avoid_distance').value
        self.avoid_turn_rate = self.get_parameter('avoid_turn_rate').value
        self.avoid_speed = self.get_parameter('avoid_speed').value
        self.bypass_speed = self.get_parameter('bypass_speed').value
        self.radius_gain = self.get_parameter('radius_gain').value
        self.heading_gain = self.get_parameter('heading_gain').value
        rate = self.get_parameter('control_rate').value

        if self.radius <= 0.0:
            raise ValueError('radius must be greater than 0')

        # ---- Fixed tuning values (change here if you want to experiment) ----
        # Hysteresis: AVOID only ends when the front is clear by this EXTRA margin.
        # Without it the robot could flip between AVOID and other states every cycle.
        self.clear_margin = 0.3          # metres
        # The "front" sector is +-30 degrees around straight ahead.
        self.front_half_angle = math.radians(30.0)
        # The "side" sectors reach from 30 to 100 degrees to the left / right.
        self.side_outer_angle = math.radians(100.0)
        # BYPASS ends when the obstacle's side is free beyond this distance...
        self.side_clear_distance = 1.5   # metres
        # ...for this long (so one lucky empty scan does not end the bypass).
        self.side_clear_time = 0.5       # seconds
        # BYPASS also keeps at least this distance to the obstacle at its side.
        self.side_min_distance = 0.5     # metres
        # Safety net: never stay in BYPASS longer than this.
        self.bypass_timeout = 10.0       # seconds
        # Never turn faster than this, and never aim more than this angle away
        # from the circle's tangent when returning to the circle.
        self.max_turn_rate = 1.0         # rad/s
        self.max_approach_angle = 0.7    # radians (about 40 degrees)

        # The circle's centre (see "IMPORTANT ASSUMPTION" at the top of the file).
        self.center_x = 0.0
        self.center_y = self.radius

        # ---- State of the node ----
        self.state = FOLLOW_CIRCLE
        self.turn_direction = 1.0        # +1 = avoiding to the left, -1 = to the right
        self.bypass_start_time = None    # when BYPASS began
        self.side_clear_since = None     # since when the obstacle's side has been free
        self.pose = None                 # latest (x, y, yaw) from odometry
        self.sectors = None              # latest (front, left, right) LIDAR distances

        # ---- Communication ----
        # Publisher: sends velocity commands to the robot.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscriptions: ROS calls the given function whenever a new message arrives.
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        # Timer: runs on_timer() `rate` times per second. This is our control loop.
        self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'Obstacle avoider started: circle radius={self.radius} m, '
            f'v={self.linear_speed} m/s, avoid_distance={self.avoid_distance} m. '
            f'Waiting for /scan and /odom ...')

    # ------------------------------------------------------------------
    # Callbacks: store the newest sensor data. The decisions are made in on_timer.
    # ------------------------------------------------------------------
    def on_scan(self, scan):
        """New LIDAR measurement: boil 360 numbers down to 3 (front, left, right)."""
        front = sector_min_range(
            scan, -self.front_half_angle, self.front_half_angle)
        left = sector_min_range(
            scan, self.front_half_angle, self.side_outer_angle)
        right = sector_min_range(
            scan, -self.side_outer_angle, -self.front_half_angle)
        self.sectors = (front, left, right)

    def on_odom(self, odom):
        """New odometry message: remember where the robot is and where it looks."""
        p = odom.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(odom.pose.pose.orientation))

    # ------------------------------------------------------------------
    # The control loop
    # ------------------------------------------------------------------
    def on_timer(self):
        """Runs 20x per second: pick a state, compute a command, send it."""
        # Do not move until both sensors have delivered data at least once.
        if self.pose is None or self.sectors is None:
            self.publish_command(0.0, 0.0)
            return

        front, left, right = self.sectors

        if self.state == FOLLOW_CIRCLE:
            # Rule: something close in front -> switch to AVOID.
            if front < self.avoid_distance:
                self.start_avoid(left, right)
                speed, turn = self.avoid_speed, self.turn_direction * self.avoid_turn_rate
            else:
                speed, turn = self.follow_circle()

        elif self.state == AVOID:
            # Rule: the way ahead is free (with a safety margin) -> switch to BYPASS.
            if front > self.avoid_distance + self.clear_margin:
                self.start_bypass()
                speed, turn = self.bypass(left, right)
            else:
                speed, turn = self.avoid_speed, self.turn_direction * self.avoid_turn_rate

        else:  # BYPASS
            if front < self.avoid_distance:
                # Another obstacle (or the same one) appeared ahead: avoid again.
                self.start_avoid(left, right)
                speed, turn = self.avoid_speed, self.turn_direction * self.avoid_turn_rate
            else:
                speed, turn = self.bypass(left, right)

        self.publish_command(speed, turn)

    # ------------------------------------------------------------------
    # The three behaviours
    # ------------------------------------------------------------------
    def follow_circle(self):
        """Compute (speed, turn rate) that keep the robot on the circle.

        Two parts:
          1. FEED-FORWARD: on a circle of radius R at speed v, the turn rate is v / R.
             This alone is what circle_driver.py does.
          2. FEEDBACK: odometry tells us where we really are, so we correct mistakes:
             - radial error = how far we are from the circle (positive = outside).
             - we aim a little INWARD if outside, OUTWARD if inside. The further
               away, the stronger the aim (but never more than max_approach_angle).
             - then we turn so that our heading matches that wanted heading.
        """
        x, y, yaw = self.pose

        # Where are we relative to the circle's centre?
        dx = x - self.center_x
        dy = y - self.center_y
        distance_from_center = math.hypot(dx, dy)
        radial_error = distance_from_center - self.radius   # + outside, - inside

        # Direction of travel along a counter-clockwise circle at this spot:
        # the angle from the centre to the robot, turned 90 degrees to the left.
        tangent_heading = math.atan2(dy, dx) + math.pi / 2.0

        # Aim a bit towards the circle if we are off it.
        correction = clamp(math.atan(self.radius_gain * radial_error),
                           -self.max_approach_angle, self.max_approach_angle)
        wanted_heading = tangent_heading + correction

        # How far is our real heading from the wanted one? (wrapped to -pi..pi)
        heading_error = wrap_angle(wanted_heading - yaw)

        turn = self.linear_speed / self.radius + self.heading_gain * heading_error
        turn = clamp(turn, -self.max_turn_rate, self.max_turn_rate)
        return self.linear_speed, turn

    def start_avoid(self, left, right):
        """Switch to AVOID and decide which way to turn (towards the freer side)."""
        # >= : if both sides are equally free we choose LEFT (towards the circle's centre).
        self.turn_direction = 1.0 if left >= right else -1.0
        side = 'left' if self.turn_direction > 0 else 'right'
        self.set_state(AVOID, f'obstacle ahead, turning {side} '
                              f'(free space: left {left:.1f} m, right {right:.1f} m)')

    def start_bypass(self):
        """Switch to BYPASS and start its timers."""
        self.bypass_start_time = self.get_clock().now()
        self.side_clear_since = None
        self.set_state(BYPASS, 'way ahead is free, passing the obstacle')

    def bypass(self, left, right):
        """Drive forward past the obstacle; return (speed, turn rate).

        If we turned LEFT to avoid, the obstacle is now on our RIGHT (and vice versa),
        so that is the side we watch. Once it is no longer there for a short while,
        we are past it and may return to the circle.
        """
        now = self.get_clock().now()
        obstacle_side_distance = right if self.turn_direction > 0 else left

        # Keep a safe distance to the obstacle: steer gently away if too close.
        turn = 0.0
        if obstacle_side_distance < self.side_min_distance:
            turn = self.turn_direction * 0.3

        # Is the obstacle's side free?
        if obstacle_side_distance > self.side_clear_distance:
            if self.side_clear_since is None:
                self.side_clear_since = now         # it just became free: start counting
            clear_for = (now - self.side_clear_since).nanoseconds / 1e9
            if clear_for >= self.side_clear_time:
                self.set_state(FOLLOW_CIRCLE, 'obstacle passed, returning to the circle')
                return self.follow_circle()
        else:
            self.side_clear_since = None            # still beside us: restart counting

        # Safety net: do not stay in BYPASS forever.
        elapsed = (now - self.bypass_start_time).nanoseconds / 1e9
        if elapsed > self.bypass_timeout:
            self.set_state(FOLLOW_CIRCLE, 'bypass timeout, returning to the circle')
            return self.follow_circle()

        return self.bypass_speed, turn

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def set_state(self, new_state, reason):
        """Change state and write one line to the terminal, so you can follow along."""
        if new_state != self.state:
            self.get_logger().info(f'{self.state} -> {new_state}: {reason}')
            self.state = new_state

    def publish_command(self, speed, turn):
        """Send one velocity command to the robot."""
        msg = Twist()
        msg.linear.x = float(speed)    # forward speed along the robot's X axis
        msg.angular.z = float(turn)    # rotation around the vertical Z axis (+ = left)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        """Send an all-zero command so the robot stops when we shut down."""
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass  # ROS may already be shutting down


def main(args=None):
    """Entry point. ROS starts the program by calling this function."""
    rclpy.init(args=args)          # 1. start ROS 2 communication
    node = ObstacleAvoider()       # 2. create our node
    try:
        rclpy.spin(node)           # 3. keep running (callbacks and timer fire) until Ctrl+C
    except KeyboardInterrupt:
        pass                       # Ctrl+C is the normal way to stop, not an error
    finally:
        node.stop_robot()          # 4. make the robot stand still
        node.destroy_node()        # 5. clean up
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

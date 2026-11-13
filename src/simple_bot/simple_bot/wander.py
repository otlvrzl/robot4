"""wander.py - drives the robot around a room and avoids walls and obstacles.

THE IDEA
--------
Unlike obstacle_avoider.py there is NO circle here. The robot simply drives
forward, and when the LIDAR sees something close, it turns away. Repeating this for
a few minutes makes the robot "wander" through the whole room. This is exactly what
we need to let SLAM (Stage 2) look at every part of the room and build a map.

The node only needs ONE sensor:
  * /scan    (LIDAR)   -> distances to the walls and obstacles around the robot.
and it sends commands to:
  * /cmd_vel           -> forward speed + turn rate (same topic as the other drivers).

A STATE MACHINE (same idea as in obstacle_avoider.py)
-----------------------------------------------------
The robot is always in exactly one state; simple rules say when to switch:

  FORWARD --(something closer than turn_distance in front)--------> TURN
  FORWARD --(something VERY close in front, emergency_distance)---> BACK_UP
  FORWARD --(every 10-25 s, for variety: a short random turn)------> TURN
  TURN    --(front is free again)----------------------------------> FORWARD
  TURN    --(turning for too long: we are probably cornered)-------> BACK_UP
  BACK_UP --(reversed for a moment, or something behind us)-------> TURN

  FORWARD : drive ahead; when a wall is close on one SIDE, steer gently away from it.
  TURN    : stand still and rotate towards the side with more free space.
  BACK_UP : reverse a little to get away from a wall / out of a corner.

Limits (good to know!):
  * The robot only reacts to what the LIDAR sees right now. It has no map and no
    plan, so it explores "randomly". Coverage is not guaranteed, but in a small room
    a few minutes are usually enough.
  * It does not use odometry, so it cannot notice if a wheel is stuck.
"""

import math
import random

# rclpy is the Python library for ROS 2.
import rclpy
# Twist = velocity command (forward speed + turn rate).
from geometry_msgs.msg import Twist
# Duration = a length of time (used to schedule the next random turn).
from rclpy.duration import Duration
from rclpy.node import Node
# Sensor data may be lost now and then; this QoS profile ("best effort") is the
# normal choice for sensors: newest data matters, old data does not.
from rclpy.qos import qos_profile_sensor_data
# LaserScan = one LIDAR measurement: a list of distances, one per laser ray.
from sensor_msgs.msg import LaserScan

# We re-use two small helper functions that already exist in obstacle_avoider.py:
#   sector_min_range(scan, low, high) -> closest distance between two angles
#   clamp(value, low, high)           -> keep a number between two limits
from simple_bot.obstacle_avoider import clamp, sector_min_range

# Names of the three states. Plain text is easy to read in the log.
FORWARD = 'FORWARD'
TURN = 'TURN'
BACK_UP = 'BACK_UP'


class Wander(Node):
    """Wanders around a room: drive forward, turn away from obstacles."""

    def __init__(self):
        super().__init__('wander')

        # ---- PARAMETERS: settings you can change without editing the code ----
        # (the launch file sets linear_speed; the rest can be changed with
        #  `ros2 param set /wander <name> <value>` or in the launch file)
        self.declare_parameter('linear_speed', 0.3)        # normal forward speed, m/s
        self.declare_parameter('turn_rate', 0.8)           # rotation speed while turning, rad/s
        self.declare_parameter('slow_distance', 1.2)       # start slowing down when front is closer, m
        self.declare_parameter('turn_distance', 0.6)       # stop and turn when front is closer, m
        self.declare_parameter('emergency_distance', 0.35) # back up when front is closer, m
        self.declare_parameter('side_distance', 0.7)       # steer away from walls closer than this, m
        self.declare_parameter('steer_gain', 1.2)          # how strongly to steer away from side walls
        self.declare_parameter('control_rate', 20.0)       # control decisions per second

        self.linear_speed = self.get_parameter('linear_speed').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.turn_distance = self.get_parameter('turn_distance').value
        self.emergency_distance = self.get_parameter('emergency_distance').value
        self.side_distance = self.get_parameter('side_distance').value
        self.steer_gain = self.get_parameter('steer_gain').value
        rate = self.get_parameter('control_rate').value

        # ---- Fixed tuning values (change here if you want to experiment) ----
        # The "front" sector is +-20 degrees around straight ahead. (A wider sector would
        # make the robot refuse to drive through gaps that are actually wide enough.)
        self.front_half_angle = math.radians(20.0)
        # The "side" sectors reach from 30 to 90 degrees to the left / right.
        self.side_outer_angle = math.radians(90.0)
        # TURN ends when the front is free by this EXTRA margin (so the robot does not
        # flip between FORWARD and TURN every cycle - this is called hysteresis).
        self.clear_margin = 0.3          # metres
        # TURN ends when the front has been free for this long (filters out one-off
        # readings, e.g. the laser beam just slipping through a gap).
        self.free_debounce = 0.2         # seconds
        # Randomness, so that the robot does NOT repeat the same path forever:
        #  * every now and then (random time between these two values) the robot makes a
        #    short turn on its own, even if nothing is in the way ...
        self.random_turn_interval = (10.0, 25.0)   # seconds between such turns
        #  * ... lasting a random time between these two values (at 0.8 rad/s,
        #    1 second is about 45 degrees).
        self.random_turn_time = (0.5, 2.5)         # seconds
        # If TURN lasts longer than this, we are probably cornered: back up.
        self.turn_timeout = 8.0          # seconds
        # BACK_UP: reverse speed, how long, and the minimal free space behind us.
        self.back_speed = 0.15           # m/s
        self.back_time = 1.5             # seconds
        self.rear_min_distance = 0.4     # metres

        # ---- State of the node ----
        self.state = FORWARD
        self.turn_direction = 1.0        # +1 = turning left, -1 = turning right
        self.state_start = None          # when the current state began
        self.turn_until = None           # end time of a random "variety" turn (or None)
        self.free_since = None           # since when the front has been free (in TURN)
        self.next_random_turn = None     # time of the next "just for variety" turn
        self.sectors = None              # latest (front, left, right, rear) distances
        self.front_left = None           # latest distance in the front-left half
        self.front_right = None          # latest distance in the front-right half

        # ---- Communication ----
        # Publisher: sends velocity commands to the robot.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscription: ROS calls on_scan() whenever a new LIDAR message arrives.
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # Timer: runs on_timer() `rate` times per second. This is our control loop.
        self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'Wander started: v={self.linear_speed} m/s, '
            f'turn_distance={self.turn_distance} m. Waiting for /scan ...')

    # ------------------------------------------------------------------
    # Callback: store the newest LIDAR data. Decisions are made in on_timer.
    # ------------------------------------------------------------------
    def on_scan(self, scan):
        """New LIDAR measurement: boil 360 numbers down to a few useful distances."""
        # Straight ahead (+-30 degrees).
        front = sector_min_range(scan, -self.front_half_angle, self.front_half_angle)
        # Whole left / right halves of the front (0 ... 90 degrees). Used to choose
        # the turn direction: we turn towards the side with MORE free space.
        left = sector_min_range(scan, 0.0, self.side_outer_angle)
        right = sector_min_range(scan, -self.side_outer_angle, 0.0)
        # Diagonal sectors (20 ... 90 degrees). Used to steer away from side walls.
        self.front_left = sector_min_range(
            scan, self.front_half_angle, self.side_outer_angle)
        self.front_right = sector_min_range(
            scan, -self.side_outer_angle, -self.front_half_angle)
        # Behind the robot (+-30 degrees around 180 degrees). The scan runs from -pi to
        # +pi, so "behind" is split into two pieces at both ends of the scan.
        rear_edge = math.pi - self.front_half_angle
        rear = min(sector_min_range(scan, rear_edge, math.pi),
                   sector_min_range(scan, -math.pi, -rear_edge))
        self.sectors = (front, left, right, rear)

    # ------------------------------------------------------------------
    # The control loop
    # ------------------------------------------------------------------
    def on_timer(self):
        """Runs 20x per second: pick a state, compute a command, send it."""
        # Do not move until the LIDAR has delivered data at least once.
        if self.sectors is None:
            self.publish_command(0.0, 0.0)
            return

        front, left, right, rear = self.sectors
        now = self.get_clock().now()
        if self.state_start is None:
            self.state_start = now
        in_state = (now - self.state_start).nanoseconds / 1e9   # seconds in this state

        if self.state == FORWARD:
            speed, turn = self.do_forward(front, left, right, now)
        elif self.state == TURN:
            speed, turn = self.do_turn(front, in_state, now)
        else:  # BACK_UP
            speed, turn = self.do_back_up(rear, in_state)

        self.publish_command(speed, turn)

    # ------------------------------------------------------------------
    # The three behaviours
    # ------------------------------------------------------------------
    def do_forward(self, front, left, right, now):
        """Drive ahead. Return (speed, turn rate)."""
        # Rule 1: wedged very close to something -> reverse first.
        if front < self.emergency_distance:
            self.set_state(BACK_UP, f'obstacle very close ({front:.2f} m), backing up')
            return -self.back_speed, 0.0

        # Rule 2: something close ahead -> stop and turn towards the freer side.
        if front < self.turn_distance:
            # Turn towards the side with more free space. If both sides are about
            # equally free (for example in front of a straight wall), pick one at random.
            if abs(left - right) < 0.2:
                self.turn_direction = random.choice([-1.0, 1.0])
            else:
                self.turn_direction = 1.0 if left > right else -1.0
            self.turn_until = None       # this is an "until the front is free" turn
            self.free_since = None
            side = 'left' if self.turn_direction > 0 else 'right'
            self.set_state(TURN, f'obstacle ahead ({front:.2f} m), turning {side} '
                                 f'(free space: left {left:.1f} m, right {right:.1f} m)')
            return 0.0, self.turn_direction * self.turn_rate

        # Rule 3: from time to time turn a bit on purpose (only if the way ahead is free).
        if self.next_random_turn is None:
            self.next_random_turn = self.time_plus(now, random.uniform(*self.random_turn_interval))
        if now > self.next_random_turn and front > self.slow_distance:
            self.next_random_turn = self.time_plus(now, random.uniform(*self.random_turn_interval))
            self.turn_direction = random.choice([-1.0, 1.0])
            # This turn simply lasts a random time, no matter what the LIDAR sees.
            self.turn_until = self.time_plus(now, random.uniform(*self.random_turn_time))
            self.free_since = None
            self.set_state(TURN, 'random turn for variety')
            return 0.0, self.turn_direction * self.turn_rate

        # Otherwise: keep driving, slower the closer the obstacle in front is.
        # slow_factor is 1.0 when far away and drops to 0.3 near turn_distance.
        slow_factor = clamp(
            (front - self.turn_distance) / (self.slow_distance - self.turn_distance),
            0.3, 1.0)
        speed = self.linear_speed * slow_factor

        # Steer gently away from walls beside us. closeness is 0 (far) ... 1 (touching).
        def closeness(distance):
            return clamp((self.side_distance - distance) / self.side_distance, 0.0, 1.0)

        # Wall close on the RIGHT -> positive value -> turn left. Wall on the left -> turn right.
        turn = self.steer_gain * (closeness(self.front_right) - closeness(self.front_left))
        return speed, turn

    def do_turn(self, front, in_state, now):
        """Rotate on the spot. Return (speed, turn rate)."""
        # Variety turn: just turn until its random end time has come.
        if self.turn_until is not None:
            if now > self.turn_until:
                self.turn_until = None
                self.set_state(FORWARD, 'random turn finished, driving on')
                return 0.0, 0.0
            return 0.0, self.turn_direction * self.turn_rate

        # Obstacle turn: turn until the way ahead is free (with a safety margin).
        if front > self.turn_distance + self.clear_margin:
            if self.free_since is None:
                self.free_since = now                      # it just became free
            free_for = (now - self.free_since).nanoseconds / 1e9
            if free_for >= self.free_debounce:             # free for long enough: go!
                self.set_state(FORWARD, 'way ahead is free, driving on')
                return 0.0, 0.0
        else:
            self.free_since = None                         # not free (any more): restart

        # Safety net: turning for too long means we are probably stuck in a corner.
        if in_state > self.turn_timeout:
            self.set_state(BACK_UP, 'turning for too long, backing up')
            return -self.back_speed, 0.0

        return 0.0, self.turn_direction * self.turn_rate

    def do_back_up(self, rear, in_state):
        """Reverse a little. Return (speed, turn rate)."""
        # Stop reversing after a while, or if there is a wall behind us.
        if in_state > self.back_time or rear < self.rear_min_distance:
            # Choose a random turn direction to get out of the dead end.
            self.turn_direction = random.choice([-1.0, 1.0])
            self.turn_until = None
            self.free_since = None
            self.set_state(TURN, 'backed up, now turning')
            return 0.0, self.turn_direction * self.turn_rate
        return -self.back_speed, 0.0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def set_state(self, new_state, reason):
        """Change state and write one line to the terminal, so you can follow along."""
        if new_state != self.state:
            self.get_logger().info(f'{self.state} -> {new_state}: {reason}')
            self.state = new_state
            self.state_start = self.get_clock().now()   # restart the "time in state" clock

    @staticmethod
    def time_plus(time, seconds):
        """Return a ROS time that is `seconds` later than `time`."""
        return time + Duration(seconds=seconds)

    def publish_command(self, speed, turn):
        """Send one velocity command to the robot."""
        msg = Twist()
        msg.linear.x = float(speed)    # forward speed along the robot's X axis (negative = reverse)
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
    node = Wander()                # 2. create our node
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

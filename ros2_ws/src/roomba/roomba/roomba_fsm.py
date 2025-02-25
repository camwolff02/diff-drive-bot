import rclpy
from rclpy.node import Node
import math
import numpy as np
import numpy.typing as npt
import transforms3d.quaternions as q 
from scipy import ndimage

from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from enum import Enum

class MinimalPublisher(Node):
    class State(Enum):
        SEARCH=0
        FORWARD=1
        BACKUP=2
        TURN=3
        RECOVERY=4

    def __init__(self):
        super().__init__('minimal_publisher')
        self.twist_publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_publisher_ = self.create_publisher(Int32, 'state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_subscription(
                LaserScan,
                "scan",
                self.lidar_listener_callback,
                5
        )
        self.create_subscription(
                Odometry,
                "odom",
                self.odom_listener_callback,
                5
        )

        self.processed_scan = None
        self.start_index, self.end_index = None, None
        self.timestamp = None
        self.orientation = None

        self.declare_parameter('fov', math.pi / 2)  # [rad]
        self.declare_parameter("forward_speed", 1.0)  # [m/s]
        self.declare_parameter("turn_speed", 1.57)  # [rad/s]
        self.declare_parameter("turn_radians", 3 * math.pi / 4)  # [rad]
        self.declare_parameter("turn_threshold", math.pi / 16)  # [rad]
        self.declare_parameter("stop_dist", 0.5)  # [m]
        self.declare_parameter("obstacle_width", 0.05)  # [m]
        self.declare_parameter("backup_time", 1.0)  # [s]
        self.declare_parameter("scan_filter_width", 2 * math.pi/180)  # [rad]
        self.declare_parameter("timeout", 3.0)  # [s]

        # declare obstacle detection window
        self._w = self.get_parameter("obstacle_width").get_parameter_value().double_value
        self._d = self.get_parameter("stop_dist").get_parameter_value().double_value
        self._inc = 0.0
        self._watchdog = self.get_clock().now()
        self.state = self.State.FORWARD

    def has_obstacle(self, scan: npt.NDArray[np.float_]) -> bool:
        self._w = self.get_parameter("obstacle_width").get_parameter_value().double_value
        self._d = self.get_parameter("stop_dist").get_parameter_value().double_value

        window = np.ones(int(self._w / (self._d * self._inc)))
        return bool(np.any(np.convolve((scan < self._d).astype(float), window) >= window.shape[0]))
    
    def timer_callback(self):
        #make sure the indexes and we have a valid array
        if (self.processed_scan is None or self.start_index is None or self.end_index is None):
            return

        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        turn_speed = self.get_parameter("turn_speed").get_parameter_value().double_value

        match self.state:
            case self.State.SEARCH:
                if not self.watchdog_safe():
                    # go to recovery state when an invalid lidar ranges array occurs
                    self.state = self.State.RECOVERY
                elif not self.has_obstacle(self.processed_scan):
                    # we found an angle
                    self.state = self.State.FORWARD
                else:
                    msg.twist.angular.z = turn_speed
               
            case self.State.FORWARD:
                if not self.watchdog_safe():
                    # go to recovery state when an invalid lidar ranges array occurs
                    self.state = self.State.RECOVERY
                elif self.has_obstacle(self.processed_scan):
                    # move forward until obstacle detected
                    self.state = self.State.BACKUP
                else:
                    msg.twist.linear.x = forward_speed

            case self.State.BACKUP:
                if self.timestamp is None:
                    self.timestamp = self.get_clock().now()
                
                # backup for one second
                diff = self.get_clock().now() - self.timestamp
                backup_ns = 1e9*self.get_parameter("backup_time").get_parameter_value().double_value
                if diff.nanoseconds > backup_ns:
                    self.timestamp = None
                    self.state = self.State.TURN
                else:
                    msg.twist.linear.x = -forward_speed * 3.0

            case self.State.TURN:
                # TODO turn to 45 degrees, then transition to SEARCH
                if self.orientation is None:
                    # wait to turn until we know our orientation
                    pass
                elif self._goal is None:
                    theta = self.get_parameter("turn_radians").get_parameter_value().double_value
                    self._goal = q.axangle2quat(self.orientation, theta)
                else:
                    _, error = q.quat2axangle(q.qmult(self._goal, q.qconjugate(self.orientation)))
                    threshold = self.get_parameter("turn_threshold").get_parameter_value().double_value
                    if error <= threshold:  # still stop if we overshoot
                        msg.twist.angular.z = turn_speed
                    else:
                        self.state = self.State.FORWARD

            case self.State.RECOVERY:
                # stay imobile until lidar online again
                if self.watchdog_safe():
                    self.state = self.State.SEARCH

        state_msg = Int32()
        state_msg.data = self.state.value
        self.state_publisher_.publish(state_msg)
        self.twist_publisher_.publish(msg)
        self.get_logger().info(f'commanding: [L: {msg.twist.linear.x}, A: {msg.twist.angular.z}]')

    def watchdog_safe(self) -> bool:
        # TODO add watchod for odometry
        timeout = 1e9*self.get_parameter("timeout").get_parameter_value().double_value
        return (self.get_clock().now() - self._watchdog).nanoseconds < timeout

    def lidar_listener_callback(self, msg: LaserScan):        
        if(self.start_index is None or self.end_index is None):
            self._inc = msg.angle_increment
            fov = self.get_parameter("fov").get_parameter_value().double_value
            self.start_index = int((len(msg.ranges) - (fov / msg.angle_increment)) // 2)
            self.end_index = int(len(msg.ranges) - self.start_index)

        # crop scan to desired fov 
        self.processed_scan = np.asarray(msg.ranges[self.start_index:self.end_index+1])

        theta = self.get_parameter("scan_filter_width").get_parameter_value().double_value
        ndimage.median_filter(self.processed_scan, int(theta / self._inc))  # reduce noise

        # pet the watchdog, we got a lidar message
        self._watchdog = self.get_clock().now()

    def odom_listener_callback(self, msg: Odometry):
        o = msg.pose.pose.orientation
        self.orientation = np.array([o.w, o.x, o.y, o.z])

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
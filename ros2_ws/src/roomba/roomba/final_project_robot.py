import rclpy
from rclpy.node import Node

import math
import numpy as np
import numpy.typing as npt
import transforms3d.quaternions as q
import transforms3d.euler as e
from scipy import ndimage

import torch
import torch.nn.functional as F

from roomba_msgs.msg import State
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, PoseArray, Point

import time


class PidController:
    def __init__(self, Kp: float, Ki: float, Kd: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self._i_error = 0
        self._previous_error = 0
        self._previous_time = 0

    def next(self, error: float) -> float:
        # calculate delta t
        now = time.time()
        dt = now - self._previous_time 
        self._previous_time = now

        # calculate derivative and integral of error
        d_error = (error - self._previous_error) / dt
        self._i_error += error
        self._previous_error = error

        # calculate and return control
        return self.Kp*error + self.Ki*self._i_error + self.Kd*d_error


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("aruco_follower")
        self.twist_publisher_ = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.state_publisher_ = self.create_publisher(State, "state", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_subscription(LaserScan, "scan", self.lidar_listener_callback, 5)
        self.create_subscription(PoseArray, "aruco_poses", self.pose_listener_callback, 5)

        self.state = State()
        self.state.data = State.FOLLOW

        self.processed_scan = None
        self.start_index, self.end_index = None, None
        self.timestamp = None
        self.orientation = None
        self.aruco_pos: Point | None = None

        self.declare_parameter("fov", math.pi / 2)  # [rad]
        self.declare_parameter("stop_dist", 0.3)  # [m]
        self.declare_parameter("scan_filter_width", 2 * math.pi / 180)  # [rad]
        self.declare_parameter("obstacle_width", 0.05)  # [m]
        self.declare_parameter("timeout", 1.00)  # [m]
        self.declare_parameter("kp_linear", 0.5)  # [m]
        self.declare_parameter("ki_linear", 0.1)  # [m]
        self.declare_parameter("kd_linear", 0.2)  # [m]
        self.declare_parameter("kp_angular", 0.5)  # [m]
        self.declare_parameter("ki_angular", 0.2)  # [m]
        self.declare_parameter("kd_angular", 0.1)  # [m]
        kp_linear = self.get_parameter("kp_linear").get_parameter_value().double_value
        ki_linear = self.get_parameter("ki_linear").get_parameter_value().double_value
        kd_linear = self.get_parameter("kd_linear").get_parameter_value().double_value
        kp_angular = self.get_parameter("kp_angular").get_parameter_value().double_value
        ki_angular = self.get_parameter("ki_angular").get_parameter_value().double_value
        kd_angular = self.get_parameter("kd_angular").get_parameter_value().double_value

        # declare obstacle detection window
        self._w = (
            self.get_parameter("obstacle_width").get_parameter_value().double_value
        )
        self._d = self.get_parameter("stop_dist").get_parameter_value().double_value
        self._inc = 0.0
        self._watchdog_lidar = self.get_clock().now()
        self.pid_linear = PidController(kp_linear, ki_linear, kd_linear)
        self.pid_angular = PidController(kp_angular, ki_angular, kd_angular)


    def has_obstacle(self, scan: npt.NDArray[np.float_]) -> bool:
        self._w = (
            self.get_parameter("obstacle_width").get_parameter_value().double_value
        )
        self._d = self.get_parameter("stop_dist").get_parameter_value().double_value

        window = np.ones(int(self._w / (self._d * self._inc * 2)))
        return bool(
            np.any(
                np.convolve((scan < self._d*2).astype(float), window) >= window.shape[0]
            )
        )


    def timer_callback(self):
        # make sure the indexes and we have a valid array
        if (
            self.processed_scan is None
            or self.start_index is None
            or self.end_index is None
            or self.aruco_pos is None
        ):
            return

        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0

        match self.state.data:
            case State.FOLLOW:
                if not self.watchdog_safe():
                    self.state.data = State.RECOVERY
                elif self.has_obstacle(self.processed_scan): #if obstacle detected
                    self.state.data = State.WAIT
                else:
                    error_l = self.aruco_pos.z - self._d
                    error_a = 0 if abs(self.aruco_pos.x) < 0.01 else self.aruco_pos.x
                    self.get_logger().info(f"{error_l}, {error_a}")
                    
                    msg.twist.linear.x = self.pid_linear.next(error_l) 
                    msg.twist.angular.z = self.pid_angular.next(error_a) 

                    # normalize
                    msg.twist.linear.x = F.sigmoid(torch.tensor(msg.twist.linear.x)).item()
                    msg.twist.angular.z = 2 * F.sigmoid(torch.tensor(msg.twist.angular.z)).item() - 1


            case State.WAIT:
                if not self.watchdog_safe():
                    self.state.data = State.RECOVERY
                elif not self.has_obstacle(self.processed_scan):  #if no obstacle in fov
                    self.state.data = State.FOLLOW

            case State.RECOVERY:
                # stay imobile until lidar online again
                if self.watchdog_safe():
                    self.state.data = State.WAIT

        self.state_publisher_.publish(self.state)
        self.twist_publisher_.publish(msg)

    def watchdog_safe(self) -> bool:
        timeout = 1e9 * self.get_parameter("timeout").get_parameter_value().double_value
        return (self.get_clock().now() - self._watchdog).nanoseconds < timeout

    def lidar_listener_callback(self, msg: LaserScan):
        if self.start_index is None or self.end_index is None:
            self._inc = msg.angle_increment
            fov = self.get_parameter("fov").get_parameter_value().double_value
            self.start_index = int((len(msg.ranges) - (fov / msg.angle_increment)) // 2)
            self.end_index = int(len(msg.ranges) - self.start_index)

        # crop scan to desired fov
        self.processed_scan = np.asarray(
            msg.ranges[self.start_index : self.end_index + 1]
        )

        theta = (
            self.get_parameter("scan_filter_width").get_parameter_value().double_value
        )
        ndimage.median_filter(
            self.processed_scan, int(theta / self._inc)
        )  # reduce noise

        # pet the watchdog, we got a lidar message
        #self._watchdog = self.get_clock().now()

    def pose_listener_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            self.aruco_pos = None
        else:
            # get the closest marker
            min_dist = math.inf
            closest_marker = Point()
            for pose in  msg.poses:
                if math.sqrt(pose.position.x**2 + pose.position.y**2) < min_dist:
                    closest_marker = pose.position
            self.aruco_pos = closest_marker
        self._watchdog = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

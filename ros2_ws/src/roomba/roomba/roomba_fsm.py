import rclpy
from rclpy.node import Node
import math
import numpy as np
import numpy.typing as npt
from scipy import ndimage

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

from enum import Enum

class MinimalPublisher(Node):
    class State(Enum):
        SPIN=0
        TURN=1
        FORWARD=2
        BACKUP=3
        RECOVERY=4

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
                LaserScan,
                "scan",
                self.lidar_listener_callback,
                5
        )

        self.processed_scan = None
        self.start_index, self.end_index = None, None

        self.declare_parameter('fov', math.pi /2)  # [rad]
        self.declare_parameter("forward_speed", 1.0)  # [m/s]
        self.declare_parameter("turn_speed", 1.57)  # [rad/s]
        self.declare_parameter("stop_dist", 0.5)  # [m]
        self.declare_parameter("obstacle_size", 5)  # idx TODO figure out math to change to [m]

        # declare obstacle detection window
        self._window = np.ones(self.get_parameter("obstacle_size").get_parameter_value().integer_value)

        self.move_foward = TwistStamped()
        self.move_foward.twist.linear.x = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.turn = TwistStamped()
        self.turn.twist.angular.z = self.get_parameter("turn_speed").get_parameter_value().double_value
        self.state = self.State.SPIN

    def has_obstacle(self, scan: npt.NDArray[np.float_]) -> bool:
        stop_dist = self.get_parameter("stop_dist").get_parameter_value().double_value
        return bool(np.any(np.convolve((scan < stop_dist).astype(float), self._window) >= self._window.shape[0]))
    
    def timer_callback(self):
        #make sure the indexes and we have a valid array
        if (self.processed_scan is None or self.start_index is None or self.end_index is None):
            return

        msg = TwistStamped()

        match self.state:
            case self.State.SPIN:
                #go to recovery state when an invalid lidar ranges array occurs

                #spin unitl we have detected free space
                #once we detect space go to TURN state
                self.state = self.State.FORWARD
                ...
            case self.State.TURN:
                # turn until facing free space
                #then move to forward state
                ...
            case self.State.FORWARD:
                #go to recovery state when an invalid lidar ranges array occurs

                # move forward until obstacle detected
                #check the lidar data for dist
                if self.has_obstacle(self.processed_scan):
                    self.state = self.State.BACKUP
                else:
                    msg = self.move_foward

            case self.State.BACKUP:
                # backup for one second
                ...
            case self.State.RECOVERY:
                # stay imobile until lidar online again
                ...
                
        self.publisher_.publish(msg)
        self.get_logger().info(f'Commanding [Linear: {msg.twist.linear.x}, Angular: {msg.twist.angular.z}]')

    def lidar_listener_callback(self, msg : LaserScan):        
        self.get_logger().info(f"angle max: {msg.angle_max}; angle min: {msg.angle_min}; angle deg: {msg.angle_increment}" +
                               f"; ranges len {len(msg.ranges)}\n ranges:{msg.ranges}")
 
        # crop scan to desired fov 
        if(self.start_index is None or self.end_index is None):
            fov = self.get_parameter("fov").get_parameter_value().double_value
            len_ranges = len(msg.ranges)
            self.start_index = int((len_ranges - (fov / msg.angle_increment)) // 2)
            self.end_index = int(len_ranges - self.start_index)

        self.processed_scan = np.asarray(msg.ranges[self.start_index:self.end_index+1])

        ndimage.median_filter(self.processed_scan, 5)  # reduce noise


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
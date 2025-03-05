import rclpy
from rclpy.node import Node
import time 


from geometry_msgs.msg import TwistStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('simon_says')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.declare_parameter("forward_speed", 0.2)  # [m/s]
        self.declare_parameter("forward_time_long", 5.0)  # [s]
        self.declare_parameter("forward_time_short", 4.0)  # [s]
        self.declare_parameter("turn_speed", 0.1)  # [s]
        self.declare_parameter("turn_time", 2.0)
        self.declare_parameter("mode", 0)

        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.forward_time_long = self.get_parameter("forward_time_long").get_parameter_value().double_value
        self.forward_time_short = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.turn_speed = self.get_parameter("turn_speed").get_parameter_value().double_value
        self.turn_time = self.get_parameter("turn_time").get_parameter_value().double_value
        self.mode = self.get_parameter("mode").get_parameter_value().integer_value

        self.create_timer(0.1, self.simon_callback)
        self.timestamp = None


    def drive_forward(self, time_: float):
        twistMsg = TwistStamped()

        if self.timestamp is None:
            self.timestamp = self.get_clock().now().nanoseconds
            twistMsg.twist.linear.x = self.forward_speed
            self.publisher_.publish(twistMsg)
        elif self.get_clock().now().nanoseconds - self.timestamp >= 1e9 * time_ :
            twistMsg.twist.linear.x = 0.0
            self.publisher_.publish(twistMsg)
            
            self.get_logger().info("Done!")
            self.destroy_node()

    def turn(self, turn_left: bool = False):
        twistMsg = TwistStamped()
        twistMsg.twist.linear.x = 0

        if self.timestamp is None:
            self.timestamp = self.get_clock().now()

            if turn_left:
                twistMsg.twist.angular.z = self.turn_speed
            else:
                twistMsg.twist.angular.z = -self.turn_speed

            self.publisher_.publish(twistMsg)

        diff = self.get_clock().now() - self.timestamp
        if diff.nanoseconds >= 1e9 * self.turn_time:
            twistMsg.twist.angular.z = 0.0
            self.publisher_.publish(twistMsg)
            self.get_logger().info("Done!")
            self.destroy_node()

       
    def simon_callback(self):
        match self.mode:
            case 0:
                self.get_logger().info("Testing short time")
                self.drive_forward(self.forward_time_short)
            case 1:
                self.get_logger().info("Testing long time")
                self.drive_forward(self.forward_time_long)
            case 2:
                self.get_logger().info("Testing turn time")
                self.turn()
    

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
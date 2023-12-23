import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import  Joy




class Joystick(Node):

    def __init__(self):
        super().__init__('joy_twist')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%d"' % msg.buttons[0])
        twist=Twist()
        twist.linear.x=msg.axes[4]*0.3
        twist.linear.y=0.0
        twist.linear.z=0.0
        twist.angular.x=0.0
        twist.angular.y=0.0
        twist.angular.z=msg.axes[3]*0.7
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    joy_node = Joystick()

    rclpy.spin(joy_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
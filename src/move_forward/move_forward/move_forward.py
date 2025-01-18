import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def move_forward(self, duration):
        # Create a Twist message to move forward
        twist = Twist()
        twist.linear.x = 0.2  # Set forward speed (adjust as needed)
        twist.angular.z = 0.0  # No rotation

        # Publish the forward command
        self.get_logger().info('Moving forward...')
        self.publisher_.publish(twist)
        
        # Wait for the specified duration
        time.sleep(duration)

        # Stop the robot
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Stopping...')

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    try:
        node.move_forward(5)  # Move forward for 5 seconds
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

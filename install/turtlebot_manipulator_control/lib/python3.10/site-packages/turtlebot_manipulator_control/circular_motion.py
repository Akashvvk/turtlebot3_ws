#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class TurtlebotManipulatorController(Node):
    def __init__(self):
        super().__init__('turtlebot_manipulator_controller')
        self.base_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = time.time()
        self.circle_angle = 0.0
        self.base_moving = True
        self.motion_complete = False

    def move_base(self, forward=True):
        msg = Twist()
        msg.linear.x = 0.2 if forward else 0.0
        self.base_publisher.publish(msg)

    def move_arm_in_circle(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()

        # Circular motion using sine and cosine
        point.positions = [
            0.5 * math.sin(self.circle_angle),               # joint1
            0.0,         # joint2
            -0.3 + 0.4 * math.cos(self.circle_angle),        # joint3
            0.0                # joint4
        ]
        point.time_from_start.sec = 1
        traj.points = [point]
        self.arm_publisher.publish(traj)

        self.circle_angle += 0.05
        if self.circle_angle >= 2 * math.pi:
            self.motion_complete = True
            self.get_logger().info('Circular motion completed')

    def control_loop(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time <= 5.0 :
            self.move_base(forward=True)
        else:
            if self.base_moving:
                self.move_base(forward=False)
                self.base_moving = False
                self.get_logger().info('Base motion complete, starting arm motion')
            if not self.motion_complete:
                self.move_arm_in_circle()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtlebotManipulatorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.move_base(forward=False)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

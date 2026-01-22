#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState

class MotionStateSubscriber(Node):
    def __init__(self):
        super().__init__('motion_state_subscriber')
        self.subscription = self.create_subscription(
            SportModeState,
            'lf/sportmodestate',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Gait: {msg.gait_type}, Height: {msg.foot_raise_height:.3f}'
        )
        self.get_logger().info(
            f'Position: x={msg.position[0]:.3f}, y={msg.position[1]:.3f}, '
            f'z={msg.position[2]:.3f}'
        )
        self.get_logger().info(
            f'Velocity: vx={msg.velocity[0]:.3f}, vy={msg.velocity[1]:.3f}, '
            f'yaw={msg.yaw_speed:.3f}'
        )
        self.get_logger().info(
            f'Body Height: {msg.body_height:.3f}'
        )
        self.get_logger().info('---')

def main(args=None):
    rclpy.init(args=args)
    node = MotionStateSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # only when rclpy still running: shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

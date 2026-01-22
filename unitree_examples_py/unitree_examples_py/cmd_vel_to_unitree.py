#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
import json
import threading

# API ID 常量 (从 ros2_sport_client.h 复制)
ROBOT_SPORT_API_ID_MOVE = 1008

class CmdVelToUnitree(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_unitree')
        
        # 订阅 cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.handle_cmd_vel, 10)
        
        # 发布 Request
        self.request_pub = self.create_publisher(
            Request, '/api/sport/request', 10)
        
        # 50Hz 控制循环
        self.timer = self.create_timer(0.02, self.send_control_command)
        
        # 状态变量
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.lock = threading.Lock()
        
        self.get_logger().info('cmd_vel_to_unitree started')
    
    def handle_cmd_vel(self, msg):
        """接收 cmd_vel"""
        with self.lock:
            self.latest_cmd = msg
            self.last_cmd_time = self.get_clock().now()
    
    def send_control_command(self):
        """定时发送控制命令"""
        with self.lock:
            # 超时保护
            elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if elapsed > 0.5:
                cmd = Twist()  # 发送零速度
            else:
                cmd = self.latest_cmd
            
            # 构造 JSON 参数
            params = {
                'x': cmd.linear.x,
                'y': cmd.linear.y,
                'z': cmd.angular.z
            }
            
            # 构造 Request 消息
            req = Request()
            req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
            req.parameter = json.dumps(params)
            
            self.request_pub.publish(req)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUnitree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
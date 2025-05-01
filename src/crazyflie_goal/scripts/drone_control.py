#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_py import Crazyswarm
import numpy as np


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Initialize Crazyswarm
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]  # Control the first drone
        
        # Create a subscriber for goal position
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_position',
            self.goal_callback,
            10
        )
        
        self.get_logger().info('Drone Controller Node has been started')
        
    def take_off(self, height=1.0):
        """Take off to specified height."""
        self.get_logger().info(f'Taking off to height: {height}m')
        self.cf.takeoff(targetHeight=height, duration=2.0)
        self.timeHelper.sleep(2.5)  # Wait for takeoff to complete
        
    def hover(self, duration=5.0):
        """Hover at current position for specified duration."""
        self.get_logger().info(f'Hovering for {duration} seconds')
        self.timeHelper.sleep(duration)
        
    def go_to_position(self, x, y, z, yaw=0):
        """Go to specified position."""
        self.get_logger().info(f'Moving to position: x={x}, y={y}, z={z}')
        pos = np.array([x, y, z])
        self.cf.goTo(pos, yaw, duration=2.0)
        self.timeHelper.sleep(2.5)  # Wait for movement to complete
        
    def land(self):
        """Land the drone."""
        self.get_logger().info('Landing')
        self.cf.land(targetHeight=0.04, duration=2.0)
        self.timeHelper.sleep(2.5)
        
    def goal_callback(self, msg):
        """Callback for receiving goal positions."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        self.get_logger().info(f'Received new goal position: x={x}, y={y}, z={z}')
        self.go_to_position(x, y, z)


def main(args=None):
    rclpy.init(args=args)
    
    controller = DroneController()
    
    try:
        # Example sequence
        controller.take_off(1.0)  # Take off to 1 meter
        controller.hover(5.0)     # Hover for 5 seconds
        
        # Spin to process callbacks (will handle goal_position messages)
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.land()
        controller.get_logger().info('Node stopped cleanly')
    
    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
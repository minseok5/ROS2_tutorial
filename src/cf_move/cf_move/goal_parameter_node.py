import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalParameterNode(Node):
    def __init__(self):
        super().__init__('goal_parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        
        # Log the declared parameters
        self.get_logger().info('Declared parameters:')
        self.get_logger().info(f'goal_x: {self.get_parameter("goal_x").value}')
        self.get_logger().info(f'goal_y: {self.get_parameter("goal_y").value}')
        self.get_logger().info(f'goal_z: {self.get_parameter("goal_z").value}')
        self.get_logger().info(f'goal_yaw: {self.get_parameter("goal_yaw").value}')
        
        # Create a subscriber to update parameters
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal',
            self.goal_callback,
            10)
        
        self.get_logger().info('Goal parameter node initialized')

    def goal_callback(self, msg):
        # Update parameters when new goal is received
        self.set_parameters([
            rclpy.Parameter('goal_x', value=msg.pose.position.x),
            rclpy.Parameter('goal_y', value=msg.pose.position.y),
            rclpy.Parameter('goal_z', value=msg.pose.position.z),
            rclpy.Parameter('goal_yaw', value=msg.pose.orientation.w)
        ])
        
        # Log the updated parameters
        self.get_logger().info('Updated parameters:')
        self.get_logger().info(f'goal_x: {self.get_parameter("goal_x").value}')
        self.get_logger().info(f'goal_y: {self.get_parameter("goal_y").value}')
        self.get_logger().info(f'goal_z: {self.get_parameter("goal_z").value}')
        self.get_logger().info(f'goal_yaw: {self.get_parameter("goal_yaw").value}')

def main(args=None):
    rclpy.init()
    node = GoalParameterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
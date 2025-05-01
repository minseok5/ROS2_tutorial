import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue

class GoalParameterClient(Node):
    def __init__(self):
        super().__init__('goal_parameter_client')
        
        # Create a timer to periodically check parameters
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Goal parameter client initialized')

    def timer_callback(self):
        try:
            # Get parameters directly using the full parameter names
            x = self.get_parameter('/goal_parameter_node/goal_x').value
            y = self.get_parameter('/goal_parameter_node/goal_y').value
            z = self.get_parameter('/goal_parameter_node/goal_z').value
            yaw = self.get_parameter('/goal_parameter_node/goal_yaw').value
            
            self.get_logger().info(f'Current goal parameters: x={x}, y={y}, z={z}, yaw={yaw}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to get parameters: {str(e)}')

def main(args=None):
    rclpy.init()
    node = GoalParameterClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
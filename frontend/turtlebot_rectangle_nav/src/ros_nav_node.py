# robot_nav_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_rectangle import RectangleNavigator  # Import the coordinate navigator class

class RobotNavNode(Node):
    def __init__(self):
        super().__init__('robot_nav_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        # Create an instance of RectangleNavigator to handle coordinates
        self.rectangle_navigator = RectangleNavigator(self)

    def timer_callback(self):
        # Get the next coordinate
        coord_info = self.rectangle_navigator.get_next_coordinate()

        # Move robot based on coordinates (this logic is simplified)
        msg = Twist()
        self.get_logger().info(f"Moving to {coord_info['label']}: {coord_info}")
        
        # Example of how robot might move (adjust as needed)
        msg.linear.x = 0.1
        msg.angular.z = 0.0

        self.publisher.publish(msg)

        # Publish the current coordinate to ROS topic
        self.rectangle_navigator.publish_coordinate(coord_info['label'])

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

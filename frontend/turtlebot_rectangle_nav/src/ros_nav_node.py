import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotNavNode(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('robot_nav_node')
        
        # Create a publisher to send messages to '/cmd_vel' topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set up a timer to call the callback function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)  # Update every 1 second
        self.counter = 0

        # Coordinates of the four corners of the rectangle (simulated longitude, latitude, altitude)
        self.coordinates = [
            ("A", -97.28322817327727, 50.12300747122199, 0),
            ("B", -97.28217525643019, 50.12171056479555, 0),
            ("C", -97.28326793963477, 50.11981930442499, 0),
            ("D", -97.2842540198655, 50.12076274108602, 0)
        ]

        # Parameters to simulate navigation behavior
        self.current_position = 0  # Start at the first corner
        self.max_position = len(self.coordinates) - 1  # Index of the last corner

    def timer_callback(self):
        # Create a Twist message to control the robot's movement
        msg = Twist()
        
        # Get the current coordinates (name, longitude, latitude, altitude)
        coord_name, longitude, latitude, altitude = self.coordinates[self.current_position]
        
        # Log the current position
        self.get_logger().info(f"Approaching {coord_name} (Longitude: {longitude}, Latitude: {latitude}, Altitude: {altitude})")
        
        # Set the linear and angular velocity for the robot
        msg.linear.x = 0.1  # Move forward with a speed of 0.1 m/s
        msg.angular.z = 0.0  # No rotation (straight movement)
        
        # Publish the Twist message to control the robot
        self.publisher.publish(msg)

        # After 10 cycles, switch to the next corner
        if self.counter >= 10:  # After 10 cycles, change the corner
            self.counter = 0
            self.current_position += 1
            if self.current_position > self.max_position:
                self.current_position = 0  # Start over from the first corner
        else:
            self.counter += 1


def main(args=None):
    # Initialize the ROS 2 communication system
    rclpy.init(args=args)
    
    # Create an instance of the RobotNavNode
    node = RobotNavNode()

    # Keep the node running and processing callbacks
    rclpy.spin(node)
    
    # Shutdown ROS 2 when done
    rclpy.shutdown()


if __name__ == '__main__':
    # Run the main function to start the node
    main()

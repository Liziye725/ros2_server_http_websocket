from rclpy.node import Node
from std_msgs.msg import String

COORDINATES = [
    ("A", -97.28322817327727, 50.12300747122199, 0),
    ("B", -97.28217525643019, 50.12171056479555, 0),
    ("C", -97.28326793963477, 50.11981930442499, 0),
    ("D", -97.28400000000000, 50.12100000000000, 0)
]

class RectangleNavigator(Node):
    def __init__(self, node):
        self.node = node
        self.publisher = self.node.create_publisher(String, '/rectangle_coordinates', 10)
        self.current_index = 0

    def publish_coordinate(self, label: str):
        coord = COORDINATES[self.current_index]
        msg = String()
        msg.data = f"{label}: {coord}"
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: {msg.data}")
        self.current_index = (self.current_index + 1) % len(COORDINATES)

    def get_next_coordinate(self):
        coord = COORDINATES[self.current_index]
        return {"label": coord[0], "longitude": coord[1], "latitude": coord[2], "altitude": coord[3]}

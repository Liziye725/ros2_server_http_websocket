import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import asyncio
import threading
from typing import List

# FastAPI app setup
app = FastAPI()

# WebSocket Manager for FastAPI
class WebSocketManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            await connection.send_json(message)

websocket_manager = WebSocketManager()

# ROS 2 Node setup
class RobotNavNode(Node):
    def __init__(self):
        super().__init__('robot_nav_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        # Coordinates of the rectangle corners
        self.coordinates = [
            ("A", -97.28322817327727, 50.12300747122199, 0),
            ("B", -97.28217525643019, 50.12171056479555, 0),
            ("C", -97.28326793963477, 50.11981930442499, 0),
            ("D", -97.2842540198655, 50.12076274108602, 0)
        ]
        self.current_position = 0
        self.max_position = len(self.coordinates) - 1

    def timer_callback(self):
        msg = Twist()

        coord_name, longitude, latitude, altitude = self.coordinates[self.current_position]
        self.get_logger().info(f"Approaching {coord_name} (Longitude: {longitude}, Latitude: {latitude}, Altitude: {altitude})")
        
        msg.linear.x = 0.1
        msg.angular.z = 0.0
        
        # Publish the movement command
        self.publisher.publish(msg)

        # After a few cycles, move to the next corner
        if self.counter >= 10:
            self.counter = 0
            self.current_position += 1
            if self.current_position > self.max_position:
                self.current_position = 0

        # Broadcast the coordinates via WebSocket
        asyncio.create_task(websocket_manager.broadcast({
            "coordinate": {
                "name": coord_name,
                "longitude": longitude,
                "latitude": latitude,
                "altitude": altitude
            }
        }))
        
        self.counter += 1

# FastAPI Route to send data to ROS 2
@app.post("/send-data")
async def send_data(data: dict):
    # Here, you would trigger ROS 2 actions based on incoming requests (optional)
    return JSONResponse(content={"status": "success", "message": "Data received by server"})

# WebSocket route to track robot location
@app.websocket("/ws/robot-location")
async def robot_location(websocket: WebSocket):
    await websocket_manager.connect(websocket)
    try:
        while True:
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        websocket_manager.disconnect(websocket)

# Function to start the ROS 2 node and FastAPI server in parallel
def start_ros_node():
    rclpy.init()
    node = RobotNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

def start_fastapi_server():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=5000)

if __name__ == '__main__':
    # Run both FastAPI server and ROS 2 node in parallel
    ros_thread = threading.Thread(target=start_ros_node)
    ros_thread.start()

    fastapi_thread = threading.Thread(target=start_fastapi_server)
    fastapi_thread.start()

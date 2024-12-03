# main.py
import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import asyncio
import threading
from websockets import WebSocketManager
from robot_nav_node import RobotNavNode  # Import the ROS 2 node

# FastAPI app setup
app = FastAPI()
websocket_manager = WebSocketManager()

# FastAPI Route to send data to ROS 2 (if needed)
@app.post("/send-data")
async def send_data(data: dict):
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
    ros_thread = threading.Thread(target=start_ros_node)
    ros_thread.start()

    fastapi_thread = threading.Thread(target=start_fastapi_server)
    fastapi_thread.start()

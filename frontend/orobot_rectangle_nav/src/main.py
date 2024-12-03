import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import asyncio
from websockets import WebSocketManager
from orobot_nav_node import ORobotNavNode
from uvicorn.config import Config
from uvicorn.server import Server

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
            await asyncio.sleep(1)  # Simulate processing of incoming messages
    except WebSocketDisconnect:
        websocket_manager.disconnect(websocket)
        print("WebSocket disconnected")
    except Exception as e:
        print(f"Unexpected error: {e}")
        await websocket.close()

# Start the ROS 2 node asynchronously
async def start_ros_node_async():
    rclpy.init()
    node = ORobotNavNode()  # My ROS 2 node is properly imported
    while rclpy.ok():
        rclpy.spin_once(node)
        await asyncio.sleep(0.5)  # Avoid blocking the event loop
    rclpy.shutdown()

# Start the FastAPI server with Uvicorn
async def start_fastapi_server_async():
    config = Config(app=app, host="0.0.0.0", port=5001, loop="asyncio")
    server = Server(config)
    await server.serve()

# Use asyncio to manage both the ROS 2 node and FastAPI server concurrently
if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.create_task(start_ros_node_async())  # Start ROS 2 node in asyncio loop
    loop.create_task(start_fastapi_server_async())  # Start FastAPI server in asyncio loop
    loop.run_forever()
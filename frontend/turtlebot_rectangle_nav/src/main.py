import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from turtlebot_rectangle_nav.ros_rectangle import RectangleNavigator
from turtlebot_rectangle_nav.websockets import WebSocketManager
import asyncio

class FastAPIServer(rclpy.node.Node):
    def __init__(self):
        super().__init__('rectangle_nav_node')
        self.app = FastAPI()
        self.navigator = RectangleNavigator(self)
        self.websocket_manager = WebSocketManager()

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @self.app.post("/send-data")
        async def send_data(data: dict):
            msg = data.get("http_message", "")
            self.navigator.publish_coordinate(msg)
            return {"status": "success", "message": f"Published: {msg}"}

        @self.app.websocket("/ws/coordinates")
        async def robot_coordinates(websocket: WebSocket):
            await self.websocket_manager.connect(websocket)
            try:
                while True:
                    coordinate = self.navigator.get_next_coordinate()
                    await self.websocket_manager.broadcast(coordinate)
                    await asyncio.sleep(1)
            except WebSocketDisconnect:
                self.websocket_manager.disconnect(websocket)

    async def run_server(self):
        import uvicorn
        config = uvicorn.Config(self.app, host="0.0.0.0", port=5001)
        server = uvicorn.Server(config)
        await server.serve()

def main(args=None):
    rclpy.init(args=args)
    server = FastAPIServer()
    asyncio.run(server.run_server())

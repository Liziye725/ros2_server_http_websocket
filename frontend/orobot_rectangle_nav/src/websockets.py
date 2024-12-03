from fastapi import WebSocket
from typing import List
import asyncio

class WebSocketManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.message_queue = []  # Queue to hold messages while clients are disconnected
        self.lock = asyncio.Lock()  # Lock for thread-safe access to shared resources

    async def connect(self, websocket: WebSocket):
        async with self.lock:
            await websocket.accept()
            self.active_connections.append(websocket)

            # Send all buffered messages to the new client
            for message in self.message_queue:
                await websocket.send_json(message)

    def disconnect(self, websocket: WebSocket):
        with self.lock:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        async with self.lock:
            if self.active_connections:
                # Send message to all active connections
                for connection in self.active_connections:
                    await connection.send_json(message)
            else:
                # If no active connections, buffer the message
                self.message_queue.append(message)

    async def clear_message_queue(self):
        # Clears the message queue if necessary (e.g., after a successful send)
        self.message_queue.clear()
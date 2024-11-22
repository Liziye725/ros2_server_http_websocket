from flask import Flask, request, jsonify, send_from_directory
import rclpy
from std_msgs.msg import String
import os
from flask_cors import CORS

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*", "allow_headers": ["Content-Type"]}})

# Initialize ROS 2 node
rclpy.init()
node = rclpy.create_node('http_ros_bridge')
publisher = node.create_publisher(String, '/http_server_test', 10)

# Handle /send-data route
@app.route('/send-data', methods=['POST'])
def receive_data():
    data = request.json  # Expecting JSON data
    print("Received data:", data)  # Print it for debugging
    msg = String()
    msg.data = data.get('http_message', '')  # Access the 'message' field
    publisher.publish(msg)
    return jsonify({"status": "success", "http_message": "Data sent to ROS 2"}), 200

# Handle /favicon.ico route
@app.route('/favicon.ico')
def favicon():
    # Serve a favicon file from a 'static' directory
    favicon_path = os.path.join(app.root_path, 'static')
    return send_from_directory(favicon_path, 'favicon.ico', mimetype='image/vnd.microsoft.icon')

if __name__ == '__main__':
    try:
        # Run the Flask app
        app.run(host='0.0.0.0', port=5001, use_reloader=False)  # Set use_reloader to False
        rclpy.spin_once(node)  # Allow ROS to process incoming callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

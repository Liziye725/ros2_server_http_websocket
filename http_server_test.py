from flask import Flask, request, jsonify
import rclpy
from std_msgs.msg import String

app = Flask(__name__)

# Initialize ROS 2 node
rclpy.init()
node = rclpy.create_node('http_ros_bridge')
publisher = node.create_publisher(String, '/http_server_test', 10)

@app.route('/send-data', methods=['POST'])
def receive_data():
    data = request.json  # Expecting JSON data
    print("Received data:", data)  # Print it for debugging
    msg = String()
    msg.data = data.get('http_message', '')  # Access the 'message' field
    publisher.publish(msg)
    return jsonify({"status": "success", "http_message": "Data sent to ROS 2"}), 200

if __name__ == '__main__':
    try:
        # Run the Flask app
        app.run(host='0.0.0.0', port=5000, use_reloader=False)  # Set use_reloader to False
        rclpy.spin_once(node)  # Allow ROS to process incoming callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

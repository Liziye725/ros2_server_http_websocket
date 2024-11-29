HTTP - not real-time

Websocket - real-time

1.download package
2.modify and build pkg
3.run launch file
4.ip a
5.ip + port




```
source ~/gnss_ws/install/setup.bash
```
### Test from postman

1. Create a virtual environment for testing HTTP
```
sudo apt install python3.10-venv
```
```
python3 -m venv path/name  # my env: cnt; pth: /home/ziye/Documents/maps/env/connect/cnt
```
```
source cnt/bin/activate
```
```
pip install Flask
```


- Installing collected packages: MarkupSafe, itsdangerous, click, blinker, Werkzeug, Jinja2, Flask
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
generate-parameter-library-py 0.3.8 requires pyyaml, which is not installed.
generate-parameter-library-py 0.3.8 requires typeguard, which is not installed.
```
pip install pyyaml typeguard
```
```
mkdir http
```
```
cd /home/ziye/Documents/maps/env/connect/http
```
```
nano http_server_test.py
```

Make sure the "http_message" aligned with your client. Also provide ip and port.
```
ip a
```

```
python http_server_test.py
```

- It should work like: (cnt) mine:~/Documents/maps/env/connect/http$ python http_server_test.py
 * Serving Flask app 'http_server_test'
 * Debug mode: off
WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:5000
 * Running on http://[server ip]:[server port]
Press CTRL+C to quit
Received data: {'status': 'active', 'battery': 85, 'temperature': 35.2}
[client ip] - - [01/Nov/2024 13:33:29] "POST /send-data?Content-Type=application/json HTTP/1.1" 200 -

### Test receive data from browser

```
pip install flask-cors
```

```
CORS(app, resources={r"/*": {"origins": "*", "allow_headers": ["Content-Type"]}})
```

#### Security
Allow firewall only on the port you are testing:
```
sudo ufw allow [port_number]
```
After usage:
```
sudo ufw enable
```
Check the staus:
```
sudo ufw status
```



For a production-grade deployment, Flask applications should run behind a WSGI server, such as:
Gunicorn (popular with Flask and Django apps)

```
pip install gunicorn
```
```
gunicorn -w 4 -b 0.0.0.0:5000 http_server_test:app
```

3. Websocket
```
sudo apt install ros-humble-rosbridge-server
```
```
ros2 pkg list | grep rosbridge_server
```
```
ros2 pkg executables rosbridge_server
```
```
ls /opt/ros/humble/share/rosbridge_server/launch
```
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
-------------

```
sudo ufw allow 9090
```
hostname -I # Or `ip a`
```

Do not use rosbridge, use foxglove:

```
sudo apt-get install ros-humble-foxglove-bridge
```
```
cd ~/gnss_ws/src
ros2 pkg create turtlebot_bringup_foxglove
cd turtlebot_bringup_foxglove
mkdir launch
```
Edit launch file and cmake file.

```
colcon build --packages-select turtlebot_bringup_foxglove
```


Check if build successfully or not:
```
ros2 pkg list 
```
```
source ~/gnss_ws/install/local_setup.bash
```
```
ros2 launch turtlebot_bringup_foxglove turtlebot_bringup_foxglove.launch
```

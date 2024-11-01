HTTP - not real-time

Websocket - real-time

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
cd /home/ziye/Documents/maps/env/connect/http
```
nano http_server_test.py
```
make sure the "http_message" aligned with your client. Also provide ip and port.
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




2. Websocket
   ```
   ros2 pkg list | grep rosbridge_server
   ```

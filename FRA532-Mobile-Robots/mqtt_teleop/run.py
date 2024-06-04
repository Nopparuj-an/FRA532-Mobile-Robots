#!/usr/bin/env python3

import os
import socket
import logging

try:
    import qrcode_terminal
    from flask import Flask, send_from_directory
except ImportError:
    os.system("pip install qrcode-terminal flask")
    import qrcode_terminal
    from flask import Flask, send_from_directory


log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

app = Flask(__name__)


@app.route("/")
def index():
    return send_from_directory(".", "index.html")


@app.route("/mqtt.min.js")
def script():
    return send_from_directory(".", "mqtt.min.js")


if __name__ == "__main__":
    # Get local IP address
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    local_ip = s.getsockname()[0]
    s.close()

    print(f"Server running at http://{local_ip}:8000/")
    qrcode_terminal.draw(f"http://{local_ip}:8000/")

    app.run(host="0.0.0.0", port=8000)

#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import ssl
import time
import random
import logging
import paho.mqtt.client as mqtt

# ========================================= Configuration =========================================

namespace = "robot1"
broker = "broker.hivemq.com"
port = 1883
username = ""
password = ""
client_id = "client-0"
version = 3  # 3 or 5
transport = "tcp"  # "tcp" or "websockets"
enable_signature = False

# ========================================= Configuration =========================================

logging.basicConfig(
    level=logging.INFO,  # DEBUG, INFO, WARNING, ERROR, CRITICAL
    format="[%(asctime)s][%(name)s][%(levelname)s]: %(message)s",
    datefmt="%H:%M:%S",  # additionally %m/%d/%Y
)

if version == 3:
    mqttc = mqtt.Client(
        client_id=client_id,
        clean_session=True,
        protocol=mqtt.MQTTv311,
        transport=transport,
    )
elif version == 5:
    mqttc = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5, transport=transport)
mqttc.username_pw_set(username, password)

# Uncomment the following line to enable TLS
# mqttc.tls_set(ca_certs="ca.crt", certfile="client.crt", keyfile="client.key", cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS, ciphers=None)


def on_connect(client, userdata, flags, reason_code, properties=None):
    logging.info(f"MQTT Connected with result code {reason_code}")
    client.subscribe(f"{namespace}/pong", qos=0)
    client.subscribe(f"{namespace}/velocities", qos=0)

vx = 0
vy = 0
omega = 0

def on_message(client, userdata, msg):
    # logging.info(f"[{msg.topic}] {str(msg.payload)}")
    try:
        if msg.topic == f"{namespace}/pong":
            data = msg.payload.decode("utf-8").split(" ")
            if data[0] != "PONG":
                return
            global last_pong_time
            last_pong_time = float(data[1])
    except:
        logging.warning("Error in receiving pong.")

    try:
        if msg.topic == f"{namespace}/velocities":
            data = msg.payload.decode("utf-8").split(" ")
            timestamp = float(data[0])
            global vx, vy, omega
            vx = float(data[1])
            vy = float(data[2])
            omega = float(data[3])
            print(f"vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}")
    except:
        logging.warning("Error in receiving velocities.")


def on_disconnect(client, userdata, reason_code, properties=None):
    logging.info(f"MQTT Disconnected with result code {reason_code}")


last_ping_time = 0
last_pong_time = 0


def ping():
    global last_ping_time
    if time.time() - last_ping_time > 0.4:
        last_ping_time = time.time()
        payload = f"PING {last_ping_time}"
        mqttc.publish(f"{namespace}/ping", payload=payload, qos=0)

    delta = last_ping_time - last_pong_time
    if delta > 2.0:
        # logging.warning(f"Connection lost for {delta:.2f} seconds.")
        return False
    else:
        return True


mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.on_disconnect = on_disconnect
mqttc.will_set(f"{namespace}/status", payload="robot_offline", qos=1, retain=False)

mqttc.connect(broker, port)
mqttc.loop_start()


class TeleopNode(Node):
    def __init__(self):
        super().__init__("mqtt_teleop_mecanum")
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.01, self.timer_callback)
        self.mqtt_last_state = False
        self.send_stop_count = 10

    def timer_callback(self):
        msg = Twist()

        connection = ping()
        if connection and not self.mqtt_last_state:
            logging.info("Connection established.")
            self.mqtt_last_state = True
        elif not connection and self.mqtt_last_state:
            logging.warning("Connection lost.")
            self.mqtt_last_state = False
            self.send_stop_count = 100

        if connection:
            msg.linear.x = vy
            msg.linear.y = -vx
            msg.angular.z = -omega
            self.pub_cmd_vel.publish(msg)
        else:
            if self.send_stop_count > 0:
                self.send_stop_count -= 1
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.pub_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

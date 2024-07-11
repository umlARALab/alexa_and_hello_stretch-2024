#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import threading

from flask import Flask, render_template, redirect, request, jsonify
import rclpy.publisher
from std_msgs.msg import String

app = Flask(__name__)

shared_data = {'message': 'Initial message'}

@app.route('/', methods=['POST', 'GET'])
def update_message():
    global shared_data
    text = 'hello world'
    shared_data['message'] = text
    return text

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.get_logger().info('Initialized')
        self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        global shared_data
        msg = String()
        msg.data = shared_data['message']
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def run_flask_server():
     app.run(port=8080)

def run_ros2_node():
     rclpy.init()
     node = MyNode()
     rclpy.spin(node)
     node.destory_node()
     rclpy.shutdown()

def main():
     flask_thread = threading.Thread(target=run_flask_server)
     flask_thread.start()

     run_ros2_node()

if __name__ == '__main__':
	main()
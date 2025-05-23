import threading
import urllib.request
import json
import time
from flask import Flask, request, jsonify, render_template, Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configure your endpoints and ports
HTTP_RECEIVE_PORT = 5000

class FlaskRos2Node(Node):
    def __init__(self):
        super().__init__('flask_ros2_node')
        # Internal list to hold QR code data for SSE
        self.qr_codes = []

        # ROS2 subscriber to listen for data from Battery node
        self.subscription = self.create_subscription(
            Array,
            'battery_node_topic',
            self.battery_node_callback,
            10
        )

        # Publisher for bluetooth controller input
        self.controller_pub = self.create_publisher(
            Array,
            'controller_input_topic',
            10
        )

        # Publisher for arrow key input
        self.keyboard_pub = self.create_publisher(
            Array,
            'keyboard_input_topic',
            10
        )

       # Publisher for automate toggle input
        self.automate_pub = self.create_publisher(
            bool,
            'automate_toggle_topic',
            10
        )

        # Initialise Flask
        self.app = Flask(__name__)

        @self.app.route('/')
        def index():
            return render_template('index.html')

        # Endpoint to handle key control input
        @self.app.route('/key_control', methods=['POST'])
        def key_control():
            data = request.get_json()
            key = data.get('key')
            # Here you can add your code to send commands to your Raspberry Pi based on the key
            print(f"Received key: {key}")
            
            # Make message and publish to keyboard input topic
            msg = String()
            msg.data = data['data']
            self.keyboard_pub.publish(msg)

            return {} #{'status': 'success', 'key': key}

        # Endpoint to handle controller input
        @self.app.route('/controller_control', methods=['POST'])
        def controller_control():
            data = request.get_json()
            R2buttons = data.get('R2')
            print(f"Received R2 buttons: {R2buttons}")

            # Make message and publish to controller input topic
            msg = String()
            msg.data = data['data']
            self.controller_pub.publish(msg)

            return {}

        # Endpoint to handle automate toggle
        @self.app.route('/automate_toggle', methods=['POST'] )
        def automate_toggle():
            data = request.get_json()
            automate = data.get('automate')
            print('Automate: ' + str(automate))

            # Make message and publish to automate toggle topic
            msg = String()
            msg.data = data['data']
            self.automate_pub.publish(msg)

            return {}

        # Continuously check for new QR codes
        @self.app.route('/qr_feed')
        def qr_feed():
            # SSE endpoint streaming new QR codes
            def event_stream():
                last_index = 0
                while True:
                    if len(self.qr_codes) > last_index:
                        code = self.qr_codes[-1]
                        yield f"data: {code}\n\n"
                        last_index = len(self.qr_codes)
                    time.sleep(0.5)
            return Response(event_stream(), mimetype='text/event-stream')

        # Run Flask in background thread
        thread = threading.Thread(
            target=self.app.run,
            kwargs={'host': '0.0.0.0', 'port': HTTP_RECEIVE_PORT}
        )
        thread.daemon = True
        thread.start()
        self.get_logger().info(f'Flask server running on port {HTTP_RECEIVE_PORT}')

    def battery_node_callback(self, msg: String):

        # Send out via HTTP POST
        try:
            payload = json.dumps({'data': msg.data}).encode('utf-8')
            req = urllib.request.Request(
                HTTP_SEND_ENDPOINT,
                data=payload,
                headers={'Content-Type': 'application/json'}
            )
            with urllib.request.urlopen(req) as resp:
                status = resp.getcode()
            self.get_logger().info(f'HTTP POST to {HTTP_SEND_ENDPOINT} returned status {status}')
        except Exception as e:
            self.get_logger().error(f'HTTP POST error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FlaskRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

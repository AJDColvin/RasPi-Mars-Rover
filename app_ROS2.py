import threading
import subprocess
import cv2
import urllib.request
import json
import time
from flask import Flask, request, jsonify, render_template, Response
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FlaskRos2Node(Node):
    def __init__(self):
        super().__init__('flask_ros2_node')
        # Internal list to hold QR code data for SSE
        self.qr_codes = []

        # Configure camera for Gstreamer/cv2 pipeline: ######
        self.camera = cv2.VideoCapture(0)

        # Initialize QR code detector
        self.detector = cv2.QRCodeDetector()

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

        self.init_Flask_app()
        threading.Thread(target=self.generate_streams, daemon=True).start()
        


    # Function for initialising Flask app and its endpoints
    def init_Flask_app(self):
        # Worried Flask won't work inside a Class!!

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
            msg = xxxxx
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
            msg = xxxxx
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
            msg = xxxx
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

        # Run Flask in separate thread
        thread = threading.Thread(
            target=self.app.run,
            kwargs={'host': '0.0.0.0', 'port': 5000}
        )
        thread.daemon = True
        thread.start()

    # Function for intitialising both low latency and OpenCV stream
    def generate_streams(self, fps=15, width=640, height=480, ip_add='127.0.0.1'):


        # Low Latency Stream FFMpeg command 
        cmd = [
            "ffmpeg",
            "-fflags", "nobuffer", "-flags", "low_delay",
            "-probesize", "32", "-analyzeduration", "0",
            "-thread_queue_size", "512",
            "-f", "v4l2",
            "-input_format", "yuyv422",
            "-video_size", f"{width}x{height}",
            "-framerate", str(fps),
            "-i", "/dev/video2",
            "-an",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-profile:v", "baseline",
            "-pix_fmt", "yuv420p",
            "-b:v", "2000k",
            "-g", str(fps),
            "-f", "rtsp",
            "rtsp://" + ip_add + ":8554/LowLat"
        ]
    
        #Run FFmpeg in command line (new process)
        subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL, start_new_session=True)

        # Define the GStreamer pipeline
        out = cv2.VideoWriter('appsrc ! videoconvert' + \
            ' ! video/x-raw,format=I420' + \
            ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=' + str(fps * 2) + \
            ' ! video/x-h264,profile=baseline' + \
            ' ! rtspclientsink location=rtsp://' + ip_add + ':8554/QRDecode',
            cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
        if not out.isOpened():
            raise Exception("can't open video writer")

        #Capture frame and look for QRs
        while True:
            ### For picamera module: ###
            # frame = camera.capture_array()
            ### For USB camera: ###
            _, frame = camera.read()
            
                # OpenCV QR code processing
            try:
                data, bbox, _ = detector.detectAndDecode(frame)
            except:
                print("An error occurred while decoding")

            if len(data)>0 and bbox is not None and len(bbox)>0:
                bbox = bbox.astype(int)

                #Blue Box around QR code
                for i in range(len(bbox[0])):
                    cv2.line(frame, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % 4]), (255, 0, 0), 2)

            # Add new QR code data only if it's different from the last scanned
            if data and (len(self.qr_codes) == 0 or data != self.qr_codes[-1]):
                self.qr_codes.append(data)
                print("QR Code Found:", data)


            out.write(frame)

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

from flask import Flask, Response, render_template, request, url_for
from picamera2 import Picamera2
import cv2


app = Flask(__name__)

camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
camera.start()

def generate_frames():
    while True:
        frame = camera.capture_array()
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Endpoint to handle key control input
@app.route('/key_control', methods=['POST'])
def key_control():
    data = request.get_json()
    key = data.get('key')
    # Here you can add your code to send commands to your Raspberry Pi based on the key
    print(f"Received key: {key}")
    # For example, you might call a function that controls motors
    return {'status': 'success', 'key': key}


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

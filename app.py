from flask import Flask, Response, render_template, request, url_for
from picamera2 import Picamera2
import cv2
import time

qr_codes = []

app = Flask(__name__)

camera = Picamera2()
camera.configure(camera.create_preview_configuration(raw={"size":(1640,1232)}, main={"format": 'XRGB8888', "size": (640, 480)}))
camera.start()

detector = cv2.QRCodeDetector()

def generate_frames():
    global qr_codes
    while True:
        frame = camera.capture_array()
        
        ###### QR Code Decoding #######
        data, bbox, _ = detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)

            for i in range(len(bbox[0])):
                cv2.line(frame, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % 4]), (255, 0, 0), 2)

            # Add new QR code data only if it's different from the last scanned
            if data and (len(qr_codes) == 0 or data != qr_codes[-1]):
                qr_codes.append(data)
                print("QR Code Found:", data)
        ###############################
        
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


# SSE endpoint to stream new QR code events
@app.route('/qr_feed')
def qr_feed():
    def event_stream():
        last_index = 0
        while True:
            if len(qr_codes) > last_index:
                # Send any new QR codes that have been added
                code = qr_codes[-1]
                yield f"data: {code}\n\n"
                last_index = len(qr_codes)
            time.sleep(0.5)
    return Response(event_stream(), mimetype="text/event-stream")


# Endpoint to handle key control input
@app.route('/key_control', methods=['POST'])
def key_control():
    data = request.get_json()
    key = data.get('key')
    # Here you can add your code to send commands to your Raspberry Pi based on the key
    print(f"Received key: {key}")
    # For example, you might call a function that controls motors
    return {} #{'status': 'success', 'key': key}

@app.route('/controller_control', methods=['POST'])
def controller_control():
    data = request.get_json()
    R2buttons = data.get('R2')

    print(f"Received R2 buttons: {R2buttons}")

    return{}


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

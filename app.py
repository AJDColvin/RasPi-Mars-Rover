from flask import Flask, Response, render_template, request, url_for
from picamera2 import Picamera2
import cv2
import time

latest_QR = ""

app = Flask(__name__)

camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (256, 144)}))
camera.start()

detector = cv2.QRCodeDetector()

def generate_frames():
    global latest_QR
    while True:
        frame = camera.capture_array()
        
        ###### QR Code Decoding #######
        
        # Detect and decode QR code
        data, bbox, _ = detector.detectAndDecode(frame)
        
        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)  # Convert to integers

            # Draw bounding box (connecting all 4 points)
            for i in range(len(bbox[0])):  # bbox[0] contains the four points
                cv2.line(frame, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % 4]), (255, 0, 0), 2)

            # Display QR Code data
            if data and data != latest_QR:
                latest_QR = data
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


@app.route('/QR_data')
def qr_feed():
    def event_stream():
        last_QR_sent = ""
        while True:
            if latest_QR != last_QR_sent:
                last_QR_sent = latest_QR
                yield f"data: {latest_QR}\n\n"
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
    return {'status': 'success', 'key': key}


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

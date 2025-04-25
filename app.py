from flask import Flask, Response, render_template, request, url_for
# from picamera2 import Picamera2
import cv2
import time
import threading
import socket

qr_codes = []

app = Flask(__name__)

##### Configure camera (Picamera): #####
# camera = Picamera2()
# camera.configure(camera.create_preview_configuration(raw={"size":(1640,1232)}, main={"format": 'RGB888', "size": (640, 480)}))
# camera.start()

###### Configure camera(USB): ######
camera = cv2.VideoCapture(0)

# Initialize QR code detector
detector = cv2.QRCodeDetector()

# MediaMTX streaming function
def generate_frames_mtx(fps=15, width=640, height=480, ip_add='127.0.0.1'):
    
    global qr_codes

    # Define the GStreamer pipeline
    out = cv2.VideoWriter('appsrc ! videoconvert' + \
        ' ! video/x-raw,format=I420' + \
        ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=' + str(fps * 2) + \
        ' ! video/x-h264,profile=baseline' + \
        ' ! rtspclientsink location=rtsp://' + ip_add + ':8554/mystream',
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
        if data and (len(qr_codes) == 0 or data != qr_codes[-1]):
            qr_codes.append(data)
            print("QR Code Found:", data)


        out.write(frame)


@app.route('/')
def index():
    ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], \
        [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]

    return render_template('index.html', ip=ip)


# SSE endpoint to stream new QR code events
@app.route('/qr_feed')
def qr_feed():
    def event_stream():
        last_index = 0
        while True:
            if len(qr_codes) > last_index:
                # Send any new QR codes that have been added
                code = qr_codes[-1]
                # 2 \n signal data is an event!
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

# Endpoint to handle controller input
@app.route('/controller_control', methods=['POST'])
def controller_control():
    data = request.get_json()
    R2buttons = data.get('R2')

    print(f"Received R2 buttons: {R2buttons}")

    return{}

# Or comment out below and use:
# gunicorn -k gevent -w 1 app:app
# In command line
if __name__ == '__main__':
    # Start generate_frames_mtx loop in a separate thread, since app.run() is blocking
    threading.Thread(target=generate_frames_mtx, daemon=True).start()

    #Start Flask app
    app.run(host='0.0.0.0', port=5000)
    
    

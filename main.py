import sys
from flask import Flask, render_template, Response, jsonify
import time
import cv2
import numpy as np
import time
from tflite_support.task import processor
from tflite_support.task import core
from tflite_support.task import vision
import paho.mqtt.client as mqtt
import logging
import threading
import RPi.GPIO as GPIO

show_video = False
stop_video = False
prev_show_video = False
show_video_lock = threading.Lock()
cap = None
topic = "detection"
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Define arrays for membrane matrix mapping
# Reference: https://www.youtube.com/watch?v=yYnX5QodqQ4
MATRIX = [
    [1, 2, 3, "A"],
    [4, 5, 6, "B"],
    [7, 8, 9, "C"],
    ["*", 0, "#", "D"]
]
ROW_PINS = [17, 18, 27, 22]
COL_PINS = [23, 25, 24, 8]

GPIO.setmode(GPIO.BCM)

app = Flask(__name__)

# Setup callbacks for MQTT Mosquitto connection
def on_message(client, userdata, message):
    global show_video
    logging.info(f"Received message on topic {message.topic}: {message.payload.decode()}")
    with show_video_lock:  # Acquire the lock before modifying show_video
        if message.payload.decode() == "motion detected":
            show_video = True
        else:
            show_video = False
            client.publish(topic, "nobody_detected", qos=0, retain=False)

def on_subscribe(client, userdata, mid, granted_qos):
    logging.info(f"Subscribed to topic with QoS {granted_qos}")

# Setup MQTT Mosquitto client
client = mqtt.Client()
client.on_message = on_message
client.on_subscribe = on_subscribe
client.username_pw_set("admin", "admin")
client.connect("raspberrypi.local", 1883, 60)
client.subscribe("motion")
client.loop_start()

# Modifies input image adding bounding boxes around the detected person
def draw_bounding_box(image, detection_info):
  has_person = False
  for detection in detection_info.detections:
    category = detection.categories[0]
    has_person = category.category_name == 'person'
    if has_person:
        bounding_box = detection.bounding_box
        box_start = bounding_box.origin_x, bounding_box.origin_y
        box_end = bounding_box.origin_x + bounding_box.width, bounding_box.origin_y + bounding_box.height
        cv2.rectangle(image, box_start, box_end, (0, 0, 255), 3) # draw red bounding box
        break
  return image, has_person

# Capture frames from camera, runs the recognition and send them to the web UI
def generate_frames():
    global cap
    has_person_array = []
    detected_person = False
    prev_has_person = False
    counter, fps = 0, 0
    start_time = time.time()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 255)  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    # Reference: https://github.com/tensorflow/examples/blob/master/lite/examples/object_detection/raspberry_pi/detect.py
    base_options = core.BaseOptions(file_name='efficientdet_lite0.tflite', use_coral=False, num_threads=4)
    detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(base_options, detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    while cap.isOpened():
        check, image = cap.read()
        if not check:
            logging.info(f"Could not read from camera due to some errors.")
            break

        image = cv2.flip(image, 1)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        detection_result = detector.detect(input_tensor)
        image, has_person = draw_bounding_box(image, detection_result)

        has_person_array.append(has_person)

        counter += 1
        if counter % fps_avg_frame_count == 0:
            end_time = time.time()
            fps = fps_avg_frame_count / (end_time - start_time)
            start_time = time.time()

            detected_person = any(has_person_array)
            if detected_person and detected_person != prev_has_person:
                client.publish(topic, "person_detected", qos=0, retain=False)
                prev_has_person = detected_person
            elif detected_person == False and detected_person != prev_has_person:
                client.publish(topic, "nobody_detected", qos=0, retain=False)
                prev_has_person = detected_person
            has_person_array.clear()

        fps_text = 'FPS = {:.1f}'.format(fps)
        text_location = (left_margin, row_size)
        cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)

        ret, buffer = cv2.imencode('.jpg', image)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

# Generate black frames to show when camera is turned off
def generate_placeholder():
    placeholder_size = (640, 480)  # Adjust the size as needed
    placeholder_image = np.zeros((placeholder_size[1], placeholder_size[0], 3), dtype=np.uint8)
    ret, buffer = cv2.imencode('.jpg', placeholder_image)
    placeholder_frame = buffer.tobytes()
    yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + placeholder_frame + b'\r\n\r\n')

# Setup the membrane matrix
def setup_matrix():
    for j in range(4):
        GPIO.setup(COL_PINS[j], GPIO.OUT)
        GPIO.output(COL_PINS[j], 1)

    for i in range(4):
        GPIO.setup(ROW_PINS[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Get pressed membrane matrix key
def get_key():
    key = None
    for j in range(4):
        GPIO.output(COL_PINS[j], 0)
        for i in range(4):
            if GPIO.input(ROW_PINS[i]) == 0:
                key = MATRIX[i][j]
                while GPIO.input(ROW_PINS[i]) == 0:
                    pass  # wait for key release
        GPIO.output(COL_PINS[j], 1)
    return key

# Check if the predefined password is entered correctly
def check_password_sequence():
    global stop_video

    password_sequence = "23AB#"
    entered_sequence = ""
    timeoutOn = False
    start_time = time.time()
    while True:
        key = get_key()
        if key:
            if not timeoutOn:
                start_time = time.time()
                timeoutOn = True
            logging.info(f"Key: {key}")
            entered_sequence += str(key)
            logging.info(f"Entered_sequence: {entered_sequence}")
            # Check if the entered sequence matches the password
            if entered_sequence == password_sequence:
                stop_video = not stop_video
                logging.info(f"sequence is correct")
                entered_sequence = ""
                timeoutOn = False

            # Reset the entered sequence if it doesn't match the password
            if not password_sequence.startswith(entered_sequence) or (timeoutOn and time.time() > start_time + 10):
                entered_sequence = ""
                logging.info(f"sequence is wrong")
                timeoutOn = False

        if timeoutOn and time.time() > start_time + 10:
            entered_sequence = ""
            logging.info(f"time out")
            timeoutOn = False

# Define webUI routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_video_frames')
def get_video_frames():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame', headers={'Cache-Control': 'no-cache'})

@app.route('/get_placeholder_frames')
def get_placeholder_frames():
    global cap
    if cap is not None:
        cap.release()
        cv2.destroyAllWindows()
    return Response(generate_placeholder(), mimetype='multipart/x-mixed-replace; boundary=frame', headers={'Cache-Control': 'no-cache'})

@app.route('/get_show_video')
def get_show_video():
    global show_video
    return jsonify(show_video=show_video)

if __name__ == '__main__':
    setup_matrix()
    keypad_thread = threading.Thread(target=check_password_sequence)
    keypad_thread.start()
    app.run(debug=True)
import sys
from flask import Flask, render_template, Response, jsonify, request
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

_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

show_video = False
stop_video = False
prev_show_video = False
show_video_lock = threading.Lock()
cap = None
topic = "detection"
logging.basicConfig(level=logging.DEBUG)

def visualize(
    image: np.ndarray,
    detection_result: processor.DetectionResult,
) -> np.ndarray:
  """Draws bounding boxes on the input image and return it.

  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualize.

  Returns:
    Image with bounding boxes.
  """
  has_person = False
  for detection in detection_result.detections:
    category = detection.categories[0]
    category_name = category.category_name
    has_person = category_name == 'person'
    if has_person:
        # Draw bounding_box
        bbox = detection.bounding_box
        start_point = bbox.origin_x, bbox.origin_y
        end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
        cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

        # Draw label and score
        
        probability = round(category.score, 2)
        result_text = category_name + ' (' + str(probability) + ')'
        text_location = (_MARGIN + bbox.origin_x,
                        _MARGIN + _ROW_SIZE + bbox.origin_y)
        cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                    _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
        break
  return image, has_person

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

client = mqtt.Client()
client.on_message = on_message
client.on_subscribe = on_subscribe
client.username_pw_set("admin", "admin")
client.connect("raspberrypi.local", 1883, 60)
client.subscribe("motion")
client.loop_start()

app = Flask(__name__)

def generate_frames():
    has_person_array = []
    detected_person = False
    prev_has_person = False
    counter, fps = 0, 0
    start_time = time.time()
    duration = 4  # seconds
    global cap
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 255)  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    base_options = core.BaseOptions(
        file_name='efficientdet_lite0.tflite', use_coral=False, num_threads=4)
    detection_options = processor.DetectionOptions(
        max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            sys.exit(
            'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )

        counter += 1
        image = cv2.flip(image, 1)

        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        detection_result = detector.detect(input_tensor)

        image, has_person = visualize(image, detection_result)
        has_person_array.append(has_person)

        if counter % fps_avg_frame_count == 0:
            end_time = time.time()
            fps = fps_avg_frame_count / (end_time - start_time)
            start_time = time.time()
            count_true = sum(has_person_array)
            count_false = len(has_person_array) - count_true
            if count_true >= 1:
                detected_person = True
            else:
                detected_person = False
            if detected_person and detected_person != prev_has_person:
                client.publish(topic, "person_detected", qos=0, retain=False)
                prev_has_person = detected_person
            elif detected_person == False and detected_person != prev_has_person:
                client.publish(topic, "nobody_detected", qos=0, retain=False)
                prev_has_person = detected_person
            has_person_array.clear()

        fps_text = 'FPS = {:.1f}'.format(fps)
        text_location = (left_margin, row_size)
        cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
            font_size, text_color, font_thickness)

        if cv2.waitKey(1) == 27 or stop_video:
            break
        #cv2.imshow('object_detector', image)
        ret, buffer = cv2.imencode('.jpg', image)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

def generate_placeholder():
    placeholder_size = (640, 480)  # Adjust the size as needed
    placeholder_image = np.zeros((placeholder_size[1], placeholder_size[0], 3), dtype=np.uint8)
    ret, buffer = cv2.imencode('.jpg', placeholder_image)
    placeholder_frame = buffer.tobytes()
    yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + placeholder_frame + b'\r\n\r\n')

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
    app.run(debug=True)
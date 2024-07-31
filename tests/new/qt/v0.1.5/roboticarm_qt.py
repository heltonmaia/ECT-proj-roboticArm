import os
import sys
import time
import webbrowser
import serial
import struct
import pyaudio
import cv2
import librosa
import numpy as np
import tensorflow as tf
from IPython.display import clear_output
from gestures import detection_infos
from gestures import robotic_arm
from gestures import available_capture_devices
from audio.file_operations import create_temp_audio_file, create_temp_image_file
from audio.audio_helpers import butter_bandpass_filter
from ultralytics import YOLO
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6 import uic
from PyQt6.QtSerialPort import QSerialPort, QSerialPortInfo

VERSION = "0.1.5"

# Default values
detected = False
in_range = False
hand_class = None
hand_position = None
state = None
direction = None
m_coord_x = 0
m_coord_y = 0
fps = 0
prev_m_coord_x = 0
prev_m_coord_y = 0
prev_frame_time = 0
new_frame_time = 0
pixel_threshold = 5
CHUNK = 2048
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
THRESHOLD = 350000
RECORD_SECONDS = 2


class RoboticArmMenu(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi('./ui/roboticarm_menu.ui', self)
        self.setWindowTitle(f'Robotic Arm Control v{VERSION}')
        self.setWindowIcon(QIcon('./src/images/roboticarm.png'))

        # Exit button closes the program
        self.exit_button.clicked.connect(self.close)

        # About button redirects to Github page
        self.about_button.clicked.connect(self.redirect_to_github)

        # Setup button loads setup window and hides the menu window
        self.setup_window = RoboticArmSetup()
        self.setup_button.clicked.connect(self.show_setup_window)

        # Start button loads execution window and hides the menu window
        self.execution_window = None
        self.start_button.clicked.connect(self.show_execution_window)

    def redirect_to_github(self):
        webbrowser.open('https://github.com/heltonmaia/ECT-proj-roboticArm/')

    def show_setup_window(self):
        self.setup_window.show()
        self.hide()

    def show_execution_window(self):
        # Reads setup values again before showing execution window
        control_method = self.setup_window.get_control_method()

        if control_method == "gestures":
            self.execution_window = RoboticArmGesturesExecution()
        elif control_method == "voice":
            self.execution_window = RoboticArmAudioExecution()
        else:
            self.execution_window = None

        if self.execution_window:
            self.execution_window.show()
            self.hide()

class RoboticArmSetup(QWidget):
    available_devices = available_capture_devices.capture_devices()

    def __init__(self):
        super().__init__()
        uic.loadUi('./ui/roboticarm_setup.ui', self)
        self.setWindowTitle(f'Robotic Arm Control v{VERSION}')
        self.setWindowIcon(QIcon('./src/images/roboticarm.png'))

        # Default values
        self.COM = 3
        self.cap_device = 0
        self.control_method = "gestures"
        self.read_setup_file()

        # Set default values for the items
        self.com_port_text.setText(str(self.COM))
        self.capture_device_text.setText(str(self.cap_device))
        self.control_method_box.addItems(["gestures", "voice"])
        self.control_method_box.setCurrentText(self.control_method)

        # Connect signals to change the default values for the items
        self.com_port_text.textChanged.connect(self.validate_com_port)
        self.capture_device_text.textChanged.connect(self.validate_capture_device)
        self.control_method_box.currentTextChanged.connect(self.update_control_method)

        # Ok button closes the setup window, saves the values and shows the menu window again
        self.ok_button.clicked.connect(self.show_menu_window)

    def read_setup_file(self):
        if os.path.exists('setup.txt'):
            with open('setup.txt', 'r') as setup_file:
                lines = setup_file.readlines()
                for line in lines:
                    line = line.strip()
                    if line.startswith('com_port='):
                        self.COM = int(line.split('=')[1])
                    elif line.startswith('capture_device='):
                        self.cap_device = int(line.split('=')[1])
                    elif line.startswith('control_method='):
                        self.control_method = str(line.split('=')[1])

    def get_capture_device(self):
        return self.cap_device

    def get_com_port(self):
        return self.COM

    def show_menu_window(self):
        if self.com_port_text.text() != '' and self.capture_device_text.text() != '':
            if int(self.capture_device_text.text()) in self.available_devices:
                menu_window.show()
                self.close()

    def validate_com_port(self, text):
        try:
            if not text:
                self.com_port_label.setStyleSheet("QLabel { color: red; }")
            elif not text.isdigit():
                self.com_port_label.setStyleSheet("QLabel { color: white; }")
                self.com_port_text.setText(text[:-1])
            else:
                self.com_port_label.setStyleSheet("QLabel { color: white; }")
                self.COM = int(text)
                self.update_setup_file()
        except Exception as e:
            print("Error while updating com_port:", e)

    def validate_capture_device(self, text):
        try:
            if not text:
                self.capture_device_label.setStyleSheet("QLabel { color: red; }")
            elif not text.isdigit():
                self.capture_device_label.setStyleSheet("QLabel { color: white; }")
                self.capture_device_text.setText(text[:-1])
            else:
                device_id = int(text)
                if device_id in self.available_devices:
                    self.capture_device_label.setStyleSheet("QLabel { color: white; }")
                    self.cap_device = int(text)
                    self.update_setup_file()
                else:
                    self.capture_device_label.setStyleSheet("QLabel { color: red; }")
                    print("Device not available")
        except Exception as e:
            print("Error while updating capture device:", e)

    def update_control_method(self, text):
        self.control_method = text
        self.update_setup_file()

    def update_setup_file(self):
        try:
            with open('setup.txt', 'w') as setup_file:
                setup_file.write(f'com_port={self.COM}\n')
                setup_file.write(f'capture_device={self.cap_device}\n')
                setup_file.write(f'control_method={self.control_method}')
        except Exception as e:
            print("Error while updating setup.txt:", e)

    def get_control_method(self):
        return self.control_method

class RoboticArmGesturesExecution(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi('./ui/roboticarm_gestures_execution.ui', self)
        self.setWindowTitle(f'Robotic Arm Control v{VERSION}')
        self.setWindowIcon(QIcon('./src/images/roboticarm.png'))

        # Exit button closes the program
        self.exit_button.clicked.connect(self.close)

        # Back button closes the execution window and shows the menu window again
        self.back_button.clicked.connect(self.show_menu_window)

        # Start/Stop button manages the camera feed
        self.start_stop_button.clicked.connect(self.start_stop_feed_slot)

        # Setting up the thread_1
        self.thread_1 = thread_1()
        self.thread_1.image_update_signal.connect(self.image_update_slot)
        self.thread_1.info_update_signal.connect(self.info_update_slot)

        # Camera feed widget
        self.camera_feed_widget = self.findChild(QWidget, 'camera_feed_widget')

        # Set initial button text and information
        if not self.thread_1.isRunning():
            self.start_stop_button.setText("Start")
            self.capture_device_info.setText("None")
            self.serial_port_info.setText("None")
            self.detected_info.setText("None")
            self.in_range_info.setText("None")
            self.position_info.setText("None")
            self.state_info.setText("None")
            self.direction_info.setText("None")
            self.fps_info.setText("None")
        else:
            self.start_stop_button.setText("Stop")

    def show_menu_window(self):
        menu_window.show()
        self.thread_1.stop()
        self.thread_1.wait()
        self.start_stop_button.setText("Start")
        self.close()


    def info_update_slot(self, capture_device, serial_port, detected, in_range, position, state, direction, fps):
        self.capture_device_info.setText(str(capture_device))
        self.serial_port_info.setText(str(serial_port))
        self.detected_info.setText(str(detected))
        self.in_range_info.setText(str(in_range))
        self.position_info.setText(str(position))
        self.state_info.setText(str(state))
        self.direction_info.setText(str(direction))
        self.fps_info.setText(str(fps))

    def image_update_slot(self, image):
        try:
            if image is not None and not image.isNull():
                self.camera_feed_widget.setPixmap(QPixmap.fromImage(image))
            else:
                print("Invalid image")
        except Exception as e:
            print("Error while updating image:", e)

    def start_stop_feed_slot(self):
        if not self.thread_1.isRunning():
            self.thread_1.start()
            self.start_stop_button.setText("Stop")
        else:
            self.thread_1.stop()
            self.start_stop_button.setText("Start")

class RoboticArmAudioExecution(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi('./ui/roboticarm_audio_execution.ui', self)
        self.setWindowTitle(f'Robotic Arm Control v{VERSION}')
        self.setWindowIcon(QIcon('./src/images/roboticarm.png'))

        # Exit button closes the program
        self.exit_button.clicked.connect(self.close)

        # Back button closes the execution window and shows the menu window again
        self.back_button.clicked.connect(self.show_menu_window)
        
        # Start/Stop button manages the audio classification
        self.start_stop_button.clicked.connect(self.start_stop_feed_slot)

        # Setting up the thread_2
        self.thread_2 = thread_2()
        self.thread_2.info_update_signal.connect(self.info_update_slot)

        # Set initial button text and information
        if not self.thread_2.isRunning():
            self.start_stop_button.setText("Start")
            self.state.setText("None")
            self.predicted_class.setText("None")
            self.confidence.setText("None")
        else:
            self.start_stop_button.setText("Stop")

    def start_stop_feed_slot(self):
        if not self.thread_2.isRunning():
            self.thread_2.start()
            self.start_stop_button.setText("Stop")
        else:
            self.thread_2.stop()
            self.start_stop_button.setText("Start")

    def show_menu_window(self):
        menu_window.show()
        self.start_stop_button.setText("Start")
        self.close()

    def info_update_slot(self, predicted_class, confidence, state):
        self.state.setText(str(state))
        self.predicted_class.setText(str(predicted_class))
        self.confidence.setText(str(confidence))


# Thread to handle the camera feed
class thread_1(QThread):

    def __init__(self):
        super().__init__()

        self.ThreadActive = True
        self.mutex = QMutex()

        if os.path.exists('setup.txt'):
            with open('setup.txt', 'r') as setup_file:
                lines = setup_file.readlines()
                for line in lines:
                    line = line.strip()
                    if line.startswith('com_port='):
                        self.COM = int(line.split('=')[1])
                    elif line.startswith('capture_device='):
                        self.cap_device = int(line.split('=')[1])
                    elif line.startswith('control_method='):
                        self.control_method = str(line.split('=')[1])
        else:
            # Default values
            self.COM = 3
            self.cap_device = 0
            self.control_method = "gestures"

    image_update_signal = pyqtSignal(QImage)
    info_update_signal = pyqtSignal(int, int, bool, bool, tuple, str, str, int)

    def run(self):

        # Serial port configuration
        try:
            self.arm = serial.Serial(f'COM{self.COM}', 9600, timeout=0.2)
            if not self.arm.isOpen():
                self.arm.open()
        except Exception as e:
            pass

        model = YOLO('./src/misc/weight-hand-segmentation-v14.pt')
        cap = cv2.VideoCapture(self.cap_device)
        prev_m_coord_x = 0
        prev_m_coord_y = 0
        prev_frame_time = 0
        new_frame_time = 0
        pixel_threshold = 5
        rtlx = detection_infos.rect_top_left_x
        rtly = detection_infos.rect_top_left_y
        rbrx = detection_infos.rect_bottom_right_x
        rbry = detection_infos.rect_bottom_right_y

        while self.ThreadActive:

            read, frame = cap.read()

            if read:

                # Uses our model in the frame
                results = model(source=frame, conf=0.6)
                boxes = results[0].boxes.cpu().numpy()

                best_detection = None
                best_score = 0

                # FPS counter
                new_frame_time = time.time()
                fps = 1 / (new_frame_time - prev_frame_time)
                prev_frame_time = new_frame_time
                fps = int(fps)

                # Process individual information for every hand in the frame
                for box in boxes:

                    if box.cls in [0, 1]:

                        # Calculates most of the needed information
                        rect_coord = box.data[0][:4]
                        conf = box.data[0][4]
                        area = detection_infos.calculate_area(rect_coord)
                        m_coord_x = int((box.xyxy[0][2] + box.xyxy[0][0]) / 2)
                        m_coord_y = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                        area_location = [int(box.xyxy[0][0]), int(box.xyxy[0][3])]
                        hand_position = (m_coord_x, m_coord_y)
                        in_range = detection_infos.is_in_range(m_coord_x, m_coord_y)
                        score = detection_infos.calculate_score(area, conf)
                        direction = detection_infos.calculate_direction(pixel_threshold, m_coord_x, m_coord_y, prev_m_coord_x, prev_m_coord_y)

                        if robotic_arm.is_moving_axis_x(pixel_threshold, prev_m_coord_x, m_coord_x):
                            rotating_base_angle = robotic_arm.calculate_rotating_base_angle(m_coord_x)
                            prev_m_coord_x = m_coord_x
                        if robotic_arm.is_moving_axis_y(pixel_threshold, prev_m_coord_y, m_coord_y):
                            arm1_angle, arm2_angle = robotic_arm.calculate_angle_hastes(m_coord_y)
                            prev_m_coord_y = m_coord_y

                        if score > best_score:
                            best_score = score
                            best_detection = box

                        if best_detection is not None:
                            detected = True
                            gripper_angle, state = robotic_arm.calculate_gripper_angle(best_detection.cls)
                            detected = True
                            if best_detection.cls == 0:
                                hand_class = 'Closed Hand'
                                state = "Closed"
                            elif best_detection.cls == 1:
                                hand_class = 'Open Hand'
                                state = "Open"
                        else:
                            detected = False
                            in_range = False
                            hand_class = 'None'
                            state = hand_class

                        if in_range:
                            self.arm.write(struct.pack('BBBB', rotating_base_angle, gripper_angle, arm1_angle, arm2_angle))

                if best_detection is None:
                    detected = False
                    in_range = False

                # Image processing
                annotated_frame = results[0].plot(boxes=False)
                flipped_annotated_frame = cv2.flip(annotated_frame, 1)
                image = cv2.cvtColor(flipped_annotated_frame, cv2.COLOR_BGR2RGB)
                flipped_image = cv2.flip(image, 1)

                try:
                    if in_range:
                        cv2.rectangle(flipped_image, (rtlx, rtly), (rbrx, rbry), (255, 0, 0), 3)
                        cv2.putText(frame, f'Area: {area:.2f}', (area_location[0], area_location[1]),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    else:
                        cv2.rectangle(flipped_image, (rtlx, rtly), (rbrx, rbry), (255, 0, 0), 3)

                    if direction != "Still":
                        cv2.circle(flipped_image, (m_coord_x, m_coord_y), 3, (0, 255, 0), 2)
                    else:
                        cv2.circle(flipped_image, (m_coord_x, m_coord_y), 3, (255, 255, 255), 2)

                except Exception as e:
                    print(e)

                converted_image = QImage(flipped_image.data,
                                         flipped_image.shape[1],
                                         flipped_image.shape[0],
                                         QImage.Format.Format_RGB888)
                final_image = converted_image.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)


                self.image_update_signal.emit(final_image)
                try:
                    self.info_update_signal.emit(self.cap_device, self.COM, detected, in_range, hand_position, state, direction, fps)
                except Exception as e:
                    print(e)
                   
        # Release the camera access
        cap.release()

    def stop(self):
        with QMutexLocker(self.mutex):
            self.ThreadActive = False

# Thread to handle the audio classification
class thread_2(QThread):

    info_update_signal = pyqtSignal(str, float, str)

    def __init__(self):
        super().__init__()
        self.commands = ["up", "left", "down", "right", "open", "close"]

        self.positions = {
            "initial": (90, 10, 120, 110),
            "left": (175, 10, 120, 140),
            "right": (5, 10, 120, 140),
            "up": (90, 10, 175, 100),
            "down": (90, 10, 105, 160),
            "upper_left": (175, 10, 170, 100),
            "lower_left": (175, 10, 100, 170),
            "upper_right": (0, 10, 100, 175),
            "lower_right": (0, 10, 170, 100)
        }

        self.current_position_key = "initial"
        self.current_position = self.positions[self.current_position_key]

        self.ThreadActive = True
        self.mutex = QMutex()
        self.state = "None"

        if os.path.exists('setup.txt'):
            with open('setup.txt', 'r') as setup_file:
                lines = setup_file.readlines()
                for line in lines:
                    line = line.strip()
                    if line.startswith('com_port='):
                        self.COM = int(line.split('=')[1])
                    elif line.startswith('capture_device='):
                        self.cap_device = int(line.split('=')[1])
                    elif line.startswith('control_method='):
                        self.control_method = str(line.split('=')[1])
        else:
            # Default values
            self.COM = 3
            self.cap_device = 0
            self.control_method = "voice"

    def run(self):

        # Serial port configuration
        try:
            self.arm = serial.Serial(f'COM{self.COM}', 9600, timeout=0.2)
            if not self.arm.isOpen():
                self.arm.open()
        except Exception as e:
            pass

        model = tf.keras.models.load_model('./src/misc/model.keras')

        def predict(model, audio_data, sample_width, channels, rate):
            self.state = "Predicting"
            audio_file = create_temp_audio_file(audio_data, sample_width, channels, rate)
            print(audio_file)
            y_1, sr_1 = librosa.load(audio_file)
            y_1_f = butter_bandpass_filter(y_1, 100, 10000, sr_1, order=5)
            y_1_f_trimmed, _ = librosa.effects.trim(y_1_f, top_db=20)
            amp_y_1 = librosa.stft(y_1_f_trimmed)
            y_1_db = librosa.amplitude_to_db(np.abs(amp_y_1), ref=np.max)

            temp_image_filename = create_temp_image_file(y_1_db)

            image = cv2.imread(temp_image_filename)
            image_tensor = tf.convert_to_tensor(image, dtype=tf.float32) / 255.0
            image_tensor = tf.expand_dims(image_tensor, axis=0)
            prediction = model.predict(image_tensor)
            confidence = np.max(prediction)
            label_pred = np.argmax(prediction, axis=1)
            command = self.commands[label_pred[0]]
            os.remove(temp_image_filename)
            os.remove(audio_file)
            return command, confidence

        def move_arm(arm: serial, COM: int, pos: tuple):
            data = struct.pack('BBBB', *pos)
            arm.write(data)

        def set_gripper_angle(current_position: tuple, predicted_class: str) -> tuple:
            if predicted_class == "open":
                self.current_position= (current_position[0], 120, current_position[2], current_position[3])
            elif predicted_class == "close":
                self.current_position= (current_position[0], 10, current_position[2], current_position[3])
            return current_position

        def voice_activity_detection():
            global start_time
            p = pyaudio.PyAudio()
            stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
            print("Listening...")
            self.state = "Listening"
            recording = False
            audio_data = b""
            while True:
                data = stream.read(CHUNK)
                fmt = f"{CHUNK}h"
                data_int = np.array(struct.unpack(fmt, data))
                energy = abs((np.sum(data_int ** 2) / len(data_int)))
                if energy > THRESHOLD:
                    print("Voice activity detected!")
                    recording = True
                    start_time = time.time()
                if recording:
                    audio_data += data
                if recording and time.time() - start_time > RECORD_SECONDS:
                    print("-> Silence <-")
                    yield audio_data
                    recording = False
                    audio_data = b""
                    clear_output(wait=True)
                if self.ThreadActive is False:
                    break
            stream.stop_stream()
            stream.close()
            p.terminate()

        for activity in voice_activity_detection():

            predicted_class, confidence = predict(model, activity, 2, CHANNELS, RATE)
            self.state = "Processing"
            self.info_update_signal.emit(predicted_class, confidence, self.state)

            if confidence >= 0.95:
                self.state = "Moving arm"
                
                while True:
                    # Trata the arm position based on the predicted class
                    if self.current_position_key == "initial":
    
                        if predicted_class == "left":
        
                            self.current_position_key = "left"
                            self.current_position = self.positions[self.current_position_key]
        
                        elif predicted_class == "right":
        
                            self.current_position_key = "right"
                            self.current_position = self.positions[self.current_position_key]
        
                        elif predicted_class == "up":
        
                            self.current_position_key = "up"
                            self.current_position = self.positions[self.current_position_key]
        
                        elif predicted_class == "down":
        
                            self.current_position_key = "down"
                            self.current_position = self.positions[self.current_position_key]
    
                        # Move the arm
                        self.current_position = set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    elif self.current_position_key == "left":
    
                        if predicted_class == "left":
                            pass
        
                        elif predicted_class == "right":
        
                            self.current_position_key = "initial"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "up":
        
                            self.current_position_key = "upper_left"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "down":
        
                            self.current_position_key = "lower_left"
                            self.current_position= self.positions[self.current_position_key]
    
                        # Move the arm
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)
    
                    elif self.current_position_key == "lower_left":

                        if predicted_class == "left":
                            pass
        
                        elif predicted_class == "right":
        
                            self.current_position_key = "down"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "up":
        
                            self.current_position_key = "left"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "down":
                            pass
    
                        # Move the arm
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    elif self.current_position_key == "upper_left":
    
                        if predicted_class == "left":
                            pass
        
                        elif predicted_class == "right":
        
                            self.current_position_key = "up"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "up":
                            pass
        
                        elif predicted_class == "down":
        
                            self.current_position_key = "left"
                            self.current_position= self.positions[self.current_position_key]
        
                        # Move the arm
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    elif self.current_position_key == "right":

                        if predicted_class == "left":
        
                            self.current_position_key = "initial"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "right":
                            pass
        
                        elif predicted_class == "up":
        
                            self.current_position_key = "upper_right"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "down":
        
                            self.current_position_key = "lower_right"
                            self.current_position= self.positions[self.current_position_key]
        
                        # Move the arm
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    elif self.current_position_key == "lower_right":
    
                        if predicted_class == "left":
        
                            self.current_position_key = "down"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "right":
                            pass
        
                        elif predicted_class == "up":
        
                            self.current_position_key = "right"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "down":
                            pass

                    elif self.current_position_key == "upper_right":

                        if predicted_class == "left":
        
                            self.current_position_key = "up"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "right":
                            pass
        
                        elif predicted_class == "up":
                            pass
        
                        elif predicted_class == "down":
        
                            self.current_position_key = "right"
                            self.current_position= self.positions[self.current_position_key]
        
                        # Move the arm
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    elif self.current_position_key == "up":
    
                        if predicted_class == "left":
        
                            self.current_position_key = "upper_left"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "right":
        
                            self.current_position_key = "upper_right"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "up":
                            pass
        
                        elif predicted_class == "down":
        
                            self.current_position_key = "initial"
                            self.current_position= self.positions[self.current_position_key]
        
                        # Move o braço
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    elif self.current_position_key == "down":

                        if predicted_class == "left":
        
                            self.current_position_key = "lower_left"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "right":
        
                            self.current_position_key = "lower_right"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "up":
        
                            self.current_position_key = "initial"
                            self.current_position= self.positions[self.current_position_key]
        
                        elif predicted_class == "down":
                            pass
    
                        # Move the arm
                        self.current_position= set_gripper_angle(self.current_position, predicted_class)
                        move_arm(self.arm, self.COM, self.current_position)

                    break

    def stop(self):
        with QMutexLocker(self.mutex):
            self.ThreadActive = False


if __name__ == '__main__':
    app = QApplication(sys.argv)
    menu_window = RoboticArmMenu()
    menu_window.show()
    try:
        sys.exit(app.exec())
    except SystemExit:
        print("Closing Window...")
        

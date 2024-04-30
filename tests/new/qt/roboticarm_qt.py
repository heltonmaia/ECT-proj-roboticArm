import sys
import time
import webbrowser
import cv2
import detection_infos
import robotic_arm
from ultralytics import YOLO
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6 import uic

# Default values
cap_device = 0
COM = 7
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

class RoboticArmMenu(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi('roboticarm_menu.ui', self)
        self.setWindowTitle('Robotic Arm Control')
        self.setWindowIcon(QIcon('roboticarm.png'))

        # Exit button closes the program
        self.exit_button.clicked.connect(self.close)

        # About button redirects to Github page
        self.about_button.clicked.connect(self.redirect_to_github)

        # Start button loads execution window and hides the menu window
        self.execution_window = RoboticArmExecution()
        self.start_button.clicked.connect(self.show_execution_window)

    def redirect_to_github(self):
        webbrowser.open('https://github.com/heltonmaia/ECT-proj-roboticArm/')

    def show_execution_window(self):
        self.execution_window.show()
        self.hide()

class RoboticArmExecution(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi('roboticarm_execution.ui', self)
        self.setWindowTitle('Robotic Arm Control')

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

# Thread to handle the camera feed
class thread_1(QThread):

    image_update_signal = pyqtSignal(QImage)
    info_update_signal = pyqtSignal(int, int, bool, bool, tuple, str, str, int)

    def run(self):

        self.ThreadActive = True
        model = YOLO('weight-hand-segmentation-v14.pt')
        cap = cv2.VideoCapture(cap_device)
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
                        hand_position = (m_coord_x, m_coord_y)
                        in_range = detection_infos.is_in_range(m_coord_x, m_coord_y)
                        score = detection_infos.calculate_score(area, conf)
                        direction = detection_infos.calculate_direction(pixel_threshold, m_coord_x, m_coord_y, prev_m_coord_x, prev_m_coord_y)

                        if robotic_arm.is_moving_axis_x(pixel_threshold, prev_m_coord_x, m_coord_x):
                            prev_m_coord_x = m_coord_x
                        if robotic_arm.is_moving_axis_y(pixel_threshold, prev_m_coord_y, m_coord_y):
                            prev_m_coord_y = m_coord_y

                        if score > best_score:
                            best_score = score
                            best_detection = box

                if best_detection is not None:
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

                # Image processing
                annotated_frame = results[0].plot(boxes=False)
                flipped_annotated_frame = cv2.flip(annotated_frame, 1)
                image = cv2.cvtColor(flipped_annotated_frame, cv2.COLOR_BGR2RGB)
                flipped_image = cv2.flip(image, 1)

                try:
                    if in_range:
                        cv2.rectangle(flipped_image, (rtlx, rtly), (rbrx, rbry), (0, 255, 0), 3)
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
                    self.info_update_signal.emit(cap_device, COM, detected, in_range, hand_position, state, direction, fps)
                except Exception as e:
                    print(e)
                #self.info_update_signal.emit(cap_device, serial_port, detected, in_range, hand_position, state, direction, fps)

    def stop(self):
        self.ThreadActive = False
        self.quit()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    menu_window = RoboticArmMenu()
    menu_window.show()
    try:
        sys.exit(app.exec())
    except SystemExit:
        print("Closing Window...")

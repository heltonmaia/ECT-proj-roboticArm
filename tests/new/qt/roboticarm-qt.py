import sys
import time
import webbrowser
import cv2
from ultralytics import YOLO
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6 import uic

def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

def calculate_score(area, conf):
    return area * conf

class RoboticArmMenu(QWidget):
    def __init__(self):
        super().__init__()
        uic.loadUi('roboticarm-menu.ui', self)
        self.setWindowTitle('Robotic Arm Control')

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
        uic.loadUi('roboticarm-execution.ui', self)
        self.setWindowTitle('Robotic Arm Control')

        # Exit button closes the program
        self.exit_button.clicked.connect(self.close)

        # Back button closes the execution window and shows the menu window again
        self.back_button.clicked.connect(self.show_menu_window)

        # Start/Stop button manages the camera feed
        self.start_stop_button.clicked.connect(self.start_stop_feed_slot)

        # Setting up the thread
        self.thread_1 = thread_1()
        self.thread_1.image_update_signal.connect(self.image_update_slot)

        # Camera feed widget
        self.camera_feed_widget = self.findChild(QWidget, 'camera_feed_widget')

        # Set initial button text
        if not self.thread_1.isRunning():
            self.start_stop_button.setText("Start")
        else:
            self.start_stop_button.setText("Stop")

    def show_menu_window(self):
        menu_window.show()
        self.thread_1.stop()
        self.start_stop_button.setText("Start")
        self.close()

    def image_update_slot(self, image):
        try:
            if image is not None and not image.isNull():
                self.camera_feed_widget.setPixmap(QPixmap.fromImage(image))
            else:
                print("Imagem inválida.")
        except Exception as e:
            print("Erro ao definir o pixmap:", e)

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

    def run(self):

        self.ThreadActive = True
        model = YOLO('weight-hand-segmentation-v14.pt')
        cap = cv2.VideoCapture(0)
        hand_class = None

        while self.ThreadActive:

            read, frame = cap.read()

            if read:

                results = model(source=frame, conf=0.6)

                boxes = results[0].boxes.cpu().numpy()

                best_detection = None
                best_score = 0

                for box in boxes:

                    if box.cls in [0, 1]:

                        rect_coord = box.data[0][:4]
                        conf = box.data[0][4]
                        area = calculateArea(rect_coord)

                        score = calculate_score(area, conf)

                        if score > best_score:
                            best_score = score
                            best_detection = box

                if best_detection is not None:

                    if best_detection.cls == 0:
                        hand_class = 'Closed Hand'

                    elif best_detection.cls == 1:
                        hand_class = 'Open Hand'

                else:
                    hand_class = 'No detection'

                # Image processing
                annotated_frame = results[0].plot(boxes=False)
                flipped_annotated_frame = cv2.flip(annotated_frame, 1)
                image = cv2.cvtColor(flipped_annotated_frame, cv2.COLOR_BGR2RGB)
                flipped_image = cv2.flip(image, 1)
                cv2.putText(flipped_image, hand_class, (10, 20), 2, 0.5, (255, 255, 255), 1)
                converted_image = QImage(flipped_image.data, flipped_image.shape[1], flipped_image.shape[0],
                                         QImage.Format.Format_RGB888)
                final_image = converted_image.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)

                # Emmiting the signal to update the camera feed with the processed image
                self.image_update_signal.emit(final_image)

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

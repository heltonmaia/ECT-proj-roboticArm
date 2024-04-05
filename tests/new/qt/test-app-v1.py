import sys
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
import cv2
from ultralytics import YOLO
import sys

def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

def calculate_score(area, conf):
    return area * conf

class mainWindow(QWidget):
    def __init__(self):
        super(mainWindow, self).__init__()
        self.setWindowTitle("Camera Feed Test")

        # Creating a box layout
        self.box_layout = QVBoxLayout()

        # Setting up the camera feed
        self.camera_feed = QLabel()
        self.box_layout.addWidget(self.camera_feed)

        # Setting up the button
        self.stop_button = QPushButton("Stop and Exit (X)")
        self.box_layout.addWidget(self.stop_button)
        self.stop_button.clicked.connect(self.cancel_feed_slot)

        # Setting up the thread
        self.thread_1 = thread_1()
        self.thread_1.start()
        self.thread_1.image_update_signal.connect(self.image_update_slot)

        # Adding the box layout to the window
        self.setLayout(self.box_layout)

    # Slot to update the camera feed
    def image_update_slot(self, image):
        self.camera_feed.setPixmap(QPixmap.fromImage(image))

    # Slot to stop the camera feed
    def cancel_feed_slot(self):
        self.thread_1.stop()

    # Stop the app by pressing X
    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_X:
            self.close()

# Retrieves frames from the camera and converting them to the PyQt format
class thread_1(QThread):

    # The signal that will be sent through the thread to our main window
    image_update_signal = pyqtSignal(QImage)

    # Function to run the thread
    def run(self):
        self.ThreadActive = True
        model = YOLO('weight-hand-segmentation-v13.pt')
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
                converted_image = QImage(flipped_image.data, flipped_image.shape[1], flipped_image.shape[0], QImage.Format.Format_RGB888)
                final_image = converted_image.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)

                # Emmiting the signal to update the camera feed with the processed image
                self.image_update_signal.emit(final_image)

    def stop(self):
        self.ThreadActive = False
        self.quit()
        sys.exit()


if __name__ == "__main__":
    App = QApplication(sys.argv)
    window = mainWindow()
    window.show()
    sys.exit(App.exec())

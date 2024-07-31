import cv2

def capture_devices():
    found_devices = []
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            found_devices.append(i)
        cap.release()
    return found_devices

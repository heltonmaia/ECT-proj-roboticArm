from ultralytics import YOLO
import cv2

def releaseConnections():
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()

def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

def calculateScore():
    return area * conf

model = YOLO('weight-hand-segmentation-v11.pt')

cap = cv2.VideoCapture(0)

while cap.isOpened():

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

                score = area

                if score > best_score:
                    best_score = score
                    best_detection = box

        if best_detection is not None:

            if best_detection.cls == 0:
                print('closed hand detected')
            elif best_detection.cls == 1:
                print('open hand detected')

        else:
            print('No detection')

        annotated_frame = results[0].plot(boxes=False)
        cv2.putText(annotated_frame, "Press x to close program", (10, 470), 2, 0.35, (255, 255, 255), 1)
        cv2.imshow('Camera Feed', annotated_frame)

        if cv2.waitKey(10) & 0xFF == ord('x'):
            break

    else:
        print('Error while reading camera feed')
        break

releaseConnections()

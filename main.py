from ultralytics import YOLO
import serial
import time
from interface import *
from detection_infos import calculate_area, calculate_direction, calculate_score, is_in_range
from robotic_arm import control_arm, calculate_rotating_base_angle, calculate_angle_hastes, is_moving_axis_x, is_moving_axis_y, calculate_gripper_angle

cap_device = 0
COM = 7
detected = False
in_range = False
hand_position = None
state = None
direction = None
m_coord_x = 0
m_coord_y = 0
fps = 0
gripper_angle = 0


model = YOLO('weight-hand-segmentation-v11.pt')
cap = cv2.VideoCapture(cap_device)

draw_home_interface()

prev_m_coord_x = 0
prev_m_coord_y = 0
prev_frame_time = 0
new_frame_time = 0
pixel_threshold = 5

def releaseConnections(cap):
    # Libera o acesso à câmera
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()

while cap.isOpened():
    read, frame = cap.read()
    frame = cv2.flip(frame, 1)

    if read:
        results = model(source=frame, conf=0.6)

        boxes = results[0].boxes.cpu().numpy()

        best_detection = None
        best_score = 0

        # FPS
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps = int(fps)

        for i, box in enumerate(boxes):
            print(enumerate(boxes))
            if box.cls in [0, 1]:
                rect_coord = box.data[0][:4]
                conf = box.data[0][4]
                area = calculate_area(rect_coord)

                m_coord_x = int((box.xyxy[0][2] + box.xyxy[0][0]) / 2)
                m_coord_y = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                in_range = is_in_range(m_coord_x, m_coord_y)

                cv2.putText(frame, f'Area {i + 1}: {area:.2f}', (int(box.xyxy[0][0]), int(box.xyxy[0][3])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                cv2.circle(frame, (m_coord_x, m_coord_y), 3, (0, 255, 0), 2)

                direction = calculate_direction(pixel_threshold, m_coord_x, m_coord_y, prev_m_coord_x, prev_m_coord_y)

                if is_moving_axis_x(pixel_threshold, prev_m_coord_x, m_coord_x):
                    rotating_base_angle = calculate_rotating_base_angle(m_coord_x)
                    prev_m_coord_x = m_coord_x
                if is_moving_axis_y(pixel_threshold, prev_m_coord_y, m_coord_y):
                    arm1_angle, arm2_angle = calculate_angle_hastes(m_coord_y)
                    prev_m_coord_y = m_coord_y

                score = calculate_score(area, conf)
                if score > best_score:
                    best_score = score
                    best_detection = box

                if best_detection is not None:
                    detected = True
                    gripper_angle, state = calculate_gripper_angle(best_detection.cls)
                if in_range:
                    control_arm(rotating_base_angle, gripper_angle, arm1_angle, arm2_angle)
        if best_detection is None:
            detected = False
            in_range = False
            print('No hand detection\n')

        # Configurações da janela principal
        window_size = (920, 500)
        window = np.zeros((window_size[1], window_size[0], 3), dtype=np.uint8)
        bg_color = (35, 15, 0)
        window[:, :] = bg_color

        # Posição e tamanho do feed de vídeo na janela
        video_position = (270, 10)
        video_size = (640, 480)

        # Desenha o feed de vídeo na posição desejada na janela e suas informações
        annotated_frame = results[0].plot(boxes=False)
        window[video_position[1]:video_position[1] + video_size[1],
        video_position[0]:video_position[0] + video_size[0]] = annotated_frame
        draw_info(m_coord_x, m_coord_y, window, cap_device, COM, detected, in_range, state, direction, fps)
        cv2.imshow('Robotic Arm Control', window)

        key = cv2.waitKey(1)

        if key & 0xFF == ord('x'):
            break
        if key & 0xFF == ord('s'):
            draw_home_interface()

    else:
        print('Error while reading camera feed')
        break

releaseConnections(cap)


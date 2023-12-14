from ultralytics import YOLO
import cv2
import sys
import numpy as np
import webbrowser
import time
import serial
import struct

def mouse_callback(event, x, y, flags, param):
    global mouse_clicked

    mouse_clicked = False

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_clicked = True
        global click_x, click_y
        click_x, click_y = x, y


def drawHomeInterface():
    window_size = (640, 480)
    window = np.zeros((window_size[1], window_size[0], 3), dtype=np.uint8)
    bg_color = (35, 15, 0)
    window[:, :] = bg_color

    title_text = "Robotic Arm Control"
    start_text = "Start"
    exit_text = "Exit"
    about_text = "About"
    shortcuts_text = "Shortcuts: (Enter -> Start), (B -> About), (X -> Exit)"

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    btn_font_scale = 1
    shortcuts_font_scale = 0.5
    font_thickness = 1
    font_color = (255, 255, 255)
    line_type = 0

    (title_text_width, _), _ = cv2.getTextSize(title_text, font, font_scale, font_thickness)
    title_position_x = int((window_size[0] - title_text_width) / 2)
    title_position_y = 100

    (start_text_width, _), _ = cv2.getTextSize(start_text, font, font_scale, font_thickness)
    start_position_x = int((window_size[0] / 2) - (start_text_width / 2))
    start_position_y = 250

    (about_text_width, _), _ = cv2.getTextSize(about_text, font, font_scale, font_thickness)
    about_position_x = int((window_size[0] / 2) - (about_text_width / 2))
    about_position_y = 300

    (exit_text_width, _), _ = cv2.getTextSize(exit_text, font, font_scale, font_thickness)
    exit_position_x = int((window_size[0] / 2) - (exit_text_width / 2))
    exit_position_y = 350

    (shortcuts_text_width, _), _ = cv2.getTextSize(shortcuts_text, font, font_scale, font_thickness)
    shortcuts_position_x = 100
    shortcuts_position_y = 450

    rect_x_min = 240
    rect_x_max = 400

    while True:
        cv2.rectangle(window, (title_position_x - 5, title_position_y - 25),
                      (title_position_x + title_text_width + 5, title_position_y + 5), (255, 255, 255), 1)
        cv2.rectangle(window, (rect_x_min, start_position_y - 30),
                      (rect_x_max, start_position_y + 10), (255, 255, 255), 1)
        cv2.rectangle(window, (rect_x_min, exit_position_y - 30),
                      (rect_x_max, exit_position_y + 10), (255, 255, 255), 1)
        cv2.rectangle(window, (rect_x_min, about_position_y - 30),
                      (rect_x_max, about_position_y + 10), (255, 255, 255), 1)

        cv2.putText(window, title_text, (title_position_x, title_position_y),
                    font, font_scale, font_color, font_thickness, cv2.LINE_AA)
        cv2.putText(window, start_text, (start_position_x, start_position_y),
                    font, btn_font_scale, font_color, font_thickness, cv2.LINE_AA)
        cv2.putText(window, exit_text, (exit_position_x, exit_position_y),
                    font, btn_font_scale, font_color, font_thickness, cv2.LINE_AA)
        cv2.putText(window, about_text, (about_position_x, about_position_y),
                    font, btn_font_scale, font_color, font_thickness, cv2.LINE_AA)
        cv2.putText(window, shortcuts_text, (shortcuts_position_x, shortcuts_position_y),
                    font, shortcuts_font_scale, font_color, font_thickness, line_type)

        cv2.namedWindow('Robotic Arm Control')
        cv2.setMouseCallback('Robotic Arm Control', mouse_callback)
        cv2.imshow('Robotic Arm Control', window)

        # Acionamento por clique do mouse
        if mouse_clicked:
            if rect_x_min <= click_x <= rect_x_max and start_position_y - 30 <= click_y <= start_position_y + 10:
                break
            elif rect_x_min <= click_x <= rect_x_max and about_position_y - 30 <= click_y <= about_position_y + 10:
                webbrowser.open('https://github.com/heltonmaia/ECT-proj-roboticArm/')
            elif rect_x_min <= click_x <= rect_x_max and exit_position_y - 30 <= click_y <= exit_position_y + 10:
                releaseConnections(cap, ser)
                break

        # Acionamento por teclas
        key = cv2.waitKey(1)
        if key == 13:
            break
        elif key == ord('b'):
            webbrowser.open('https://github.com/heltonmaia/ECT-proj-roboticArm/')
        elif key == ord('x'):
            releaseConnections(cap, ser)
            break


def drawInfo(window):
    global rect_top_left_x, rect_top_left_y, rect_bottom_right_x, rect_bottom_right_y
    rect_top_left_x = 0
    rect_top_left_y = 0
    rect_bottom_right_x = 0
    rect_bottom_right_y = 0

    x_min = 30
    x_max = 250
    y_min = 120
    y_max = 350

    hand_position = (m_coord_x, m_coord_y)

    # Cantos do retângulo
    rect_top_left_x = 380
    rect_top_left_y = 90
    rect_bottom_right_x = 800
    rect_bottom_right_y = 410

    info_text = "INFO"
    capture_device_text = "Capture Device:"
    serial_port_text = "Serial Port (COM):"
    detected_text = "Detected:"
    in_range_text = "In range:"
    position_text = "Position:"
    state_text = "State:"
    direction_text = "Direction:"
    stop_text = "Stop(S)"
    exit_text = "Exit(X)"

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    font_thickness = 1
    font_color = (255, 255, 255)

    # Informações estáticas
    cv2.putText(window, info_text, (x_min + 80, 80), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, capture_device_text, (x_min, 150), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, serial_port_text, (x_min, 180), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, detected_text, (x_min, 210), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, in_range_text, (x_min, 240), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, position_text, (x_min, 270), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, state_text, (x_min, 300), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, direction_text, (x_min, 330), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, stop_text, (x_min + 5, 400), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, exit_text, (x_min + 140, 400), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.rectangle(window, (x_min - 10, y_min), (x_max, y_max), (255, 255, 255), 1)
    cv2.rectangle(window, (x_min - 5, y_max + 25), (x_max - 135, y_max + 65), (255, 255, 255), 1)
    cv2.rectangle(window, (x_min + 125, y_max + 25), (x_max - 5, y_max + 65), (255, 255, 255), 1)

    # Informações dinâmicas
    cv2.putText(window, str(cap_device), (x_max - 70, 150), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(COM), (x_max - 45, 180), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(detected), (x_max - 130, 210), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(in_range), (x_max - 133, 240), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(hand_position), (x_max - 138, 270), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(state), (x_max - 163, 300), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(direction), (x_max - 130, 330), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, f'fps: {str(fps)}', (275, 30), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    if in_range:
        cv2.rectangle(window, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                      (0, 255, 0), 3)
    else:
        cv2.rectangle(window, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                      (0, 0, 255), 3)


def releaseConnections(cap, ser):
    # Libera o acesso à câmera
    if cap.isOpened():
        cap.release()
    if ser.is_open:
        ser.close()
    cv2.destroyAllWindows()

def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])


def calculateScore(area, conf):
    return area * conf


def calculateDirection(curr_x, curr_y, prev_x, prev_y):
    global pixel_threshold
    # Variação de posição
    delta_x = curr_x - prev_x
    delta_y = curr_y - prev_y

    # Limite mínimo de pixels para detectar o movimento
    pixel_threshold = 5

    # Limite mínimo para detectar movimentos diagonais
    diagonal_threshold = pixel_threshold * 0.7071  # 0.7071 é a raiz quadrada de 2 dividida por 2

    if delta_x > pixel_threshold:
        if delta_y > diagonal_threshold:
            return "l left"
        elif delta_y < -diagonal_threshold:
            return "u left"
        else:
            return "left"
    elif delta_x < -pixel_threshold:
        if delta_y > diagonal_threshold:
            return "l right"
        elif delta_y < -diagonal_threshold:
            return "u right"
        else:
            return "right"

    if delta_y > pixel_threshold:
        return "down"
    elif delta_y < -pixel_threshold:
        return "up"

    return "still"


def posicao_braco(ser, angle_base, angle_garra, angle_haste1, angle_haste2):
    # ser.write(struct.pack('BBBB', angle_base, angle_garra, angle_haste1, angle_haste2))
    ser.write(struct.pack('BBBB', angle_base, angle_garra, angle_haste1, angle_haste2))

def calculate_angle_base(m_coord_x):
    #angle_base = m_coord_x*180//600
    angle_base = map(m_coord_x, 110, 530, 0, 180)
    return int(angle_base)

def calculate_angle_hastes(m_coord_y):
    #angle_haste2 = m_coord_y*180//480
    angle_haste2 = map(m_coord_y, 80, 400, 95, 180)
    aux_h1 = (180*95) //angle_haste2
    #angle_haste1 = 180 - angle_haste2
    #angle_haste1 = map(angle_haste1, 0, 180, 95, 180)


    return int(aux_h1), int(angle_haste2)

def map(value, min_val,max_val, min_target, max_target):
    return np.interp(value, [min_val, max_val], [min_target, max_target])

ser = serial.Serial('COM7', 9600, timeout=0.2)
if not ser.isOpen():
    ser.open()


global cap_device, COM, detected, in_range, hand_position, state, direction, frame_w, frame_h, m_coord_x, m_coord_y, fps, angle_garra
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
angle_garra = 0

'''
# Tenta criar um objeto para a porta serial
try:
    ser = serial.Serial(f"COM{COM}", 9600)
    ser.open()
except serial.SerialException:
    print('Conectando...')
'''

model = YOLO('weight-hand-segmentation-v8.pt')
cap = cv2.VideoCapture(cap_device)

mouse_clicked = False
drawHomeInterface()

prev_m_coord_x = 0
prev_m_coord_y = 0
prev_frame_time = 0
new_frame_time = 0

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

            if box.cls in [0, 1]:

                rect_coord = box.data[0][:4]
                conf = box.data[0][4]
                area = calculateArea(rect_coord)

                m_coord_x = int((box.xyxy[0][2] + box.xyxy[0][0]) / 2)
                m_coord_y = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                if m_coord_x > rect_top_left_x - 240 and m_coord_x < rect_bottom_right_x - 240 and m_coord_y > rect_top_left_y + 30 and m_coord_y < rect_bottom_right_y - 40:
                    in_range = True
                else:
                    in_range = False

                cv2.putText(frame, f'Area {i + 1}: {area:.2f}', (int(box.xyxy[0][0]), int(box.xyxy[0][3])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                cv2.circle(frame, (m_coord_x, m_coord_y), 3, (0, 255, 0), 2)

                direction = calculateDirection(m_coord_x, m_coord_y, prev_m_coord_x, prev_m_coord_y)

                if abs(prev_m_coord_x - m_coord_x) > pixel_threshold:
                    angle_base = calculate_angle_base(m_coord_x)
                    prev_m_coord_x = m_coord_x
                if abs(prev_m_coord_y - m_coord_y) > pixel_threshold:
                    angle_haste1, angle_haste2 = calculate_angle_hastes(m_coord_y)
                    prev_m_coord_y = m_coord_y

                score = calculateScore(area, conf)
                if score > best_score:
                    best_score = score
                    best_detection = box

                if best_detection is not None:
                    detected = True
                    if best_detection.cls == 0:
                        state = "closed"
                        angle_garra = 10
                    elif best_detection.cls == 1:
                        state = "open"
                        angle_garra = 120

                if in_range:
                    posicao_braco(ser, angle_base, angle_garra, angle_haste1, angle_haste2)






            print(best_detection.cls)

        else:
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
        drawInfo(window)
        cv2.imshow('Robotic Arm Control', window)

        key = cv2.waitKey(1)

        if key & 0xFF == ord('x'):
            break
        if key & 0xFF == ord('s'):
            drawHomeInterface()

    else:
        print('Error while reading camera feed')
        break

releaseConnections(cap, ser)

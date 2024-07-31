import numpy
import numpy as np
import cv2
import webbrowser
from detection_infos import *

mouse_clicked = 0
click_x = 0
click_y = 0


def mouse_callback(event: int, x: float, y: float, flags: int, params: any):
    global mouse_clicked, click_x, click_y
    mouse_clicked = False
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_clicked = True
        click_x, click_y = x, y


def draw_home_interface():
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
                # release_connections(cap, ser)
                break

        # Acionamento por teclas
        key = cv2.waitKey(1)
        if key == 13:
            break
        elif key == ord('b'):
            webbrowser.open('https://github.com/heltonmaia/ECT-proj-roboticArm/')
        elif key == ord('x'):
            # release_connections(cap, ser)
            break


def draw_info(m_coord_x: float, m_coord_y: float, window, cap_device: int, COM: int, detected: bool, in_range: bool,
              state: str, direction: str, fps: int):
    x_min = 30
    x_max = 250
    y_min = 120
    y_max = 350

    hand_position = (m_coord_x, m_coord_y)

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
    cv2.putText(window, str(hand_position), (x_max - 138, 270), font, font_scale, font_color, font_thickness,
                cv2.LINE_AA)
    cv2.putText(window, str(state), (x_max - 163, 300), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, str(direction), (x_max - 130, 330), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    cv2.putText(window, f'fps: {str(fps)}', (275, 30), font, font_scale, font_color, font_thickness, cv2.LINE_AA)
    if in_range:
        cv2.rectangle(window, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                      (0, 255, 0), 3)
    else:
        cv2.rectangle(window, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                      (0, 0, 255), 3)


def text_area(frame: any, i: int, area: float, b1: any, b2: any):
    cv2.putText(frame, f'Area {i + 1}: {area:.2f}', (b1, b2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def draw_center(frame: any, m_coord_x: int, m_coord_y: int):
    cv2.circle(frame, (m_coord_x, m_coord_y), 3, (0, 255, 0), 2)


def window_config(results: any):
    window_size = (920, 500)
    window = np.zeros((window_size[1], window_size[0], 3), dtype=np.uint8)
    bg_color = (35, 15, 0)
    window[:, :] = bg_color

    video_position = (270, 10)
    video_size = (640, 480)

    annotated_frame = results
    window[video_position[1]:video_position[1] + video_size[1],
    video_position[0]:video_position[0] + video_size[0]] = annotated_frame
    return window


def releaseConnections(cap):
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()

import struct
import serial
import numpy as np
from typing import Tuple, Any
from numpy import ndarray, dtype, floating, float_
from numpy._typing import _64Bit

def control_arm(COM: int, rotating_base_angle: int, gripper_angle: int, arm1_angle: int, arm2_angle: int):
    #ser = serial.Serial(f"COM{str(COM)}", 9600)
    #ser.open()
    #ser.write(struct.pack('BBBB', rotating_base_angle, gripper_angle, arm1_angle, arm2_angle))
    print(f"{COM}, {rotating_base_angle}, {gripper_angle}, {arm1_angle}, {arm2_angle}")

def calculate_rotating_base_angle(m_coord_x: float) -> int:
    rotating_base_angle = map(m_coord_x, 110, 530, 0, 180)
    return int(rotating_base_angle)


def calculate_angle_hastes(m_coord_y: float) -> Tuple[int, int]:
    arm2_angle = map(m_coord_y, 80, 400, 95, 180)
    aux_h1 = (180 * 95) // arm2_angle
    return int(aux_h1), int(arm2_angle)


def calculate_gripper_angle(garra_state: bool) -> Tuple[int, str]:
    if garra_state == 0:
        return 10, "Closed"
    elif garra_state == 1:
        return 120, "Open"


def is_moving_axis_x(pixel_threshold: int, prev_m_coord_x: float, m_coord_x: float) -> bool:
    if abs(prev_m_coord_x - m_coord_x) > pixel_threshold:
        return True
    return False


def is_moving_axis_y(pixel_threshold: int, prev_m_coord_y: float, m_coord_y: float) -> bool:
    if abs(prev_m_coord_y - m_coord_y) > pixel_threshold:
        return True
    return False


def map(value: float, min_val: float, max_val: float, min_target: float, max_target: float) -> ndarray[
    Any, dtype[floating[_64Bit] | float_]]:
    return np.interp(value, [min_val, max_val], [min_target, max_target])

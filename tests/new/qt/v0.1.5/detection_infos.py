rect_top_left_x = 100
rect_top_left_y = 90
rect_bottom_right_x = 540
rect_bottom_right_y = 390

def calculate_area(rect: list[float]) -> float:
    return (rect[2] - rect[0]) * (rect[3] - rect[1])


def calculate_score(area: float, conf: float) -> float:
    return area * conf


def calculate_direction(pixel_threshold: int, curr_x: float, curr_y: float, prev_x: float, prev_y: float) -> str:
    # Variação de posição
    delta_x = curr_x - prev_x
    delta_y = curr_y - prev_y

    # Limite mínimo para detectar movimentos diagonais
    diagonal_threshold = pixel_threshold * 0.7071  # 0.7071 é a raiz quadrada de 2 dividida por 2

    if delta_x > pixel_threshold:
        if delta_y > diagonal_threshold:
            return "Lower left"
        elif delta_y < -diagonal_threshold:
            return "Upper left"
        else:
            return "Left"
    elif delta_x < -pixel_threshold:
        if delta_y > diagonal_threshold:
            return "Lower right"
        elif delta_y < -diagonal_threshold:
            return "Upper right"
        else:
            return "Right"

    if delta_y > pixel_threshold:
        return "Down"
    elif delta_y < -pixel_threshold:
        return "Up"

    return "Still"

def is_in_range(m_coord_x: float, m_coord_y: float) -> bool:
    if m_coord_x > rect_top_left_x and m_coord_x < rect_bottom_right_x and m_coord_y > rect_top_left_y and m_coord_y < rect_bottom_right_y:
        return True
    else:
        return False

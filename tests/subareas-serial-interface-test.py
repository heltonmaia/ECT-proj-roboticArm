import mediapipe as mp
import cv2
import sys
import numpy as np
import serial

# Função de liberar conexões
def releaseConnections(cap, ser):
    # Libera o acesso à câmera
    if cap.isOpened():
        cap.release()

    # Fecha a conexão serial
    if ser.is_open:
        ser.close()

    cv2.destroyAllWindows()

# Função da mão aberta
def aberta():
    try:
        cv2.putText(image, "|O|", (70, 60), 2, 0.35, (255, 0, 0), 1)
        ser.write(b'1')
    except:
        releaseConnections(cap)
        sys.exit(1)

# Função da mão fechada
def fechada():
    try:
        cv2.putText(image, "|C|", (70, 60), 2, 0.35, (255, 0, 0), 1)
        ser.write(b'0')
    except:
        releaseConnections(cap)
        sys.exit(1)

# Função da mão acima
def acima():
    try:
        cv2.putText(image, "|U|", (85, 60), 2, 0.35, (255, 0, 0), 1)

        if m_coord_y_float >= 0.25 and m_coord_y_float < 0.33:
            cv2.putText(image, "|U1|", (90, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 41
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_y_float >= 0.33 and m_coord_y_float < 0.41:
            cv2.putText(image, "|U2|", (90, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 42
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_y_float >= 0.41 and m_coord_y_float <= 0.5:
            cv2.putText(image, "|U3|", (90, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 43
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

    except:
        releaseConnections(cap)
        sys.exit(1)

# Função da mão abaixo
def abaixo():
    try:
        cv2.putText(image, "|D|", (85, 60), 2, 0.35, (255, 0, 0), 1)

        if m_coord_y_float > 0.5 and m_coord_y_float <= 0.58:
            cv2.putText(image, "|D1|", (90, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 51
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_y_float > 0.58 and m_coord_y_float <= 0.63:
            cv2.putText(image, "|D2|", (90, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 52
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_y_float > 0.63 and m_coord_y_float <= 0.75:
            cv2.putText(image, "|D3|", (90, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 53
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

    except:
        releaseConnections(cap)
        sys.exit(1)

# Função da mão à direita
def direita():
    try:
        cv2.putText(image, "|R|", (100, 60), 2, 0.35, (255, 0, 0), 1)

        if m_coord_x_float >= 0.25 and m_coord_x_float < 0.3125:
            cv2.putText(image, "|R1|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 21
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_x_float >= 0.3125 and m_coord_x_float < 0.375:
            cv2.putText(image, "|R2|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 22
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_x_float >= 0.375 and m_coord_x_float < 0.4375:
            cv2.putText(image, "|R3|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 23
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_x_float >= 0.4375 and m_coord_x_float <= 0.5:
            cv2.putText(image, "|R4|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 24
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

    except:
        releaseConnections(cap)
        sys.exit(1)

# Função da mão à esquerda
def esquerda():
    try:
        cv2.putText(image, "|L|", (100, 60), 2, 0.35, (255, 0, 0), 1)

        if m_coord_x_float > 0.5 and m_coord_x_float <= 0.5625:
            cv2.putText(image, "|L1|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 31
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_x_float > 0.5625 and m_coord_x_float <= 0.625:
            cv2.putText(image, "|L2|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 32
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_x_float > 0.625 and m_coord_x_float <= 0.6875:
            cv2.putText(image, "|L3|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 33
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

        if m_coord_x_float > 0.6875 and m_coord_x_float <= 0.75:
            cv2.putText(image, "|L4|", (70, 70), 2, 0.35, (255, 0, 0), 1)
            bt = 34
            btconv = bt.to_bytes(1, 'big')
            ser.write(btconv)

    except:
        releaseConnections(cap)
        sys.exit(1)

# Função de identificação de setor mais externo
def sectorSelection():
    if m_coord_x_float <= 0.5:
        
        if m_coord_y_float <= 0.5:
            direita(ser)
            acima(ser)
        elif m_coord_y_float > 0.5:
            direita(ser)
            abaixo(ser)
            
    elif m_coord_x_float > 0.5:
        
        if m_coord_y_float <= 0.5:
            esquerda(ser)
            acima(ser)
        elif m_coord_y_float > 0.5:
            esquerda(ser)
            abaixo(ser)
            

mp_drawing = mp.solutions.drawing_utils

mp_holistic = mp.solutions.holistic

cap = cv2.VideoCapture(0)

# Tenta se conectar a porta serial
try:
    ser = serial.Serial("COM8", 9600, timeout=0.01)
    ser.open()
except serial.SerialException:
    print('Conectando...')

# Inicializa o modelo holístico e dá um nickname
with mp_holistic.Holistic(min_detection_confidence=0.5,
                          min_tracking_confidence=0.5) as holistic:
    while cap.isOpened():
        read, frame = cap.read()

        if not read:
            break

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = holistic.process(image)

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Obtém a resolução da imagem, define altura e largura
        resolution = image.shape[:2]
        height = resolution[0]
        width = resolution[1]

        # Cantos do retângulo
        rect_top_left_x = int(width / 2) - 200
        rect_top_left_y = int(height / 2) - 150
        rect_bottom_right_x = int(width / 2) + 200
        rect_bottom_right_y = int(height / 2) + 150

        # Informações estáticas
        cv2.putText(image, "--- INFO ----", (10, 20), 2, 0.35, (0, 0, 0), 1)
        cv2.putText(image, "DETECTED:", (10, 30), 2, 0.35, (0, 0, 0), 1)
        cv2.putText(image, "IN RANGE:", (10, 40), 2, 0.35, (0, 0, 0), 1)
        cv2.putText(image, "POSITION:", (10, 50), 2, 0.35, (0, 0, 0), 1)
        cv2.putText(image, "COMMAND:", (10, 60), 2, 0.35, (0, 0, 0), 1)
        cv2.putText(image, "SUBAREA:", (10, 70), 2, 0.35, (0, 0, 0), 1)

        # Detecta se a mão direita foi lida corretamente
        if results.right_hand_landmarks:

            # Desenha um retângulo no frame processado
            cv2.rectangle(image, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                          (0, 0, 255), 3)
            
            # Alguma mão direita detectada na imagem
            cv2.putText(image, "TRUE", (70, 30), 2, 0.35, (255, 0, 0), 1)

            # Distância auxiliar para começar a detectar a mão (em pixels)
            safe_distance = 35

            # Define o range em x e em y
            range_left = rect_top_left_x + safe_distance
            range_right = rect_bottom_right_x - safe_distance + 20
            range_top = rect_top_left_y + safe_distance
            range_bottom = rect_bottom_right_y - (safe_distance * 2)

            # Média das posições de cada dedo da mão
            landmarks_array = results.right_hand_landmarks.landmark
            coord_landmarks_x = [landmark.x for landmark in landmarks_array]
            coord_landmarks_y = [landmark.y for landmark in landmarks_array]
            m_coord_x_float = np.mean(coord_landmarks_x)
            m_coord_x = int(width * m_coord_x_float)
            m_coord_y_float = np.mean(coord_landmarks_y)
            m_coord_y = int(height * m_coord_y_float)
            
            # Posição do ponto médio
            cv2.putText(image, f"({m_coord_x}, {m_coord_y})", (70, 50), 2, 0.35, (0, 0, 0), 1)

            if (m_coord_x >= range_left and m_coord_x <= range_right) and (
                       m_coord_y >= range_top and m_coord_y <= range_bottom):

                # Mão detectada dentro do alcance "in range"
                cv2.putText(image, "TRUE", (70, 40), 2, 0.35, (255, 0, 0), 1)

                mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                          mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=1, circle_radius=1),
                                          mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=1, circle_radius=1))

                # Mão detectada dentro do alcance "in range"
                cv2.rectangle(image, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                              (0, 255, 0), 3)

                # Ponto médio das coordenadas dos dedos
                cv2.circle(image, (m_coord_x, m_coord_y), 3, (0, 0, 255), 2)

                # Dedos indicador e polegar
                ind = results.right_hand_landmarks.landmark[8]
                pol = results.right_hand_landmarks.landmark[4]

                dist2center = 0.08

                # Distância minima para encontro dos dedos
                distmin = 0.03

                # Limites esquerdo e direito
                leftL = m_coord_x - dist2center
                rightL = m_coord_x + dist2center

                # Detecta se a mão está fechada ou aberta
                if abs(ind.x - pol.x) <= distmin:
                    fechada(ser)
                else:
                    aberta(ser)

                # Posição da mão nos eixos x e y
                sectorSelection()
            else:

                cv2.putText(image, "FALSE", (70, 40), 2, 0.35, (0, 0, 255), 1)
                cv2.putText(image, "NONE", (70, 60), 2, 0.35, (0, 0, 255), 1)
                cv2.putText(image, "NONE", (70, 70), 2, 0.35, (0, 0, 255), 1)

        else:

            cv2.putText(image, "FALSE", (70, 30), 2, 0.35, (0, 0, 255), 1)
            cv2.putText(image, "FALSE", (70, 40), 2, 0.35, (0, 0, 255), 1)
            cv2.putText(image, "( - , -)", (70, 50), 2, 0.35, (0, 0, 0), 1)
            cv2.putText(image, "NONE", (70, 60), 2, 0.35, (0, 0, 255), 1)
            cv2.putText(image, "NONE", (70, 70), 2, 0.35, (0, 0, 255), 1)

        # Exibe o frame processado
        cv2.imshow('Camera Feed', image)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

releaseConnections(cap, ser)

from ultralytics import YOLO
import cv2
import sys
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
def aberta(ser):
    try:
       ser.write(b'1')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão fechada
def fechada(ser):
    try:
        ser.write(b'0')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão à direita
def direita(ser):
    try:
        ser.write(b'2')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão à esquerda
def esquerda(ser):
    try:
        ser.write(b'3')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão acima
def acima(ser):
    try:
        ser.write(b'4')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão abaixo
def abaixo(ser):
    try:
        ser.write(b'5')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão parada
def parada(ser):
    try:
        ser.write(b'6')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função para calcular a área da bounding box
def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

# Função para calcular score
def calculateScore(area, conf):
    return area * conf

# Função para calcular a direção do movimento da mão
def calculate_direction(curr_x, curr_y, prev_x, prev_y):

    # Variação de posição
    delta_x = curr_x - prev_x
    delta_y = curr_y - prev_y

    # Limite mínimo de pixels para detectar o movimento
    pixel_threshold = 5

    # Limite mínimo para detectar movimentos diagonais
    diagonal_threshold = pixel_threshold * 0.7071  # 0.7071 é a raiz quadrada de 2 dividida por 2

    if delta_x > pixel_threshold:
        if delta_y > diagonal_threshold:
            return "diagonal inferior esquerda"
        elif delta_y < -diagonal_threshold:
            return "diagonal superior esquerda"
        else:
            return "esquerda"
    elif delta_x < -pixel_threshold:
        if delta_y > diagonal_threshold:
            return "diagonal inferior direita"
        elif delta_y < -diagonal_threshold:
            return "diagonal superior direita"
        else:
            return "direita"

    if delta_y > pixel_threshold:
        return "abaixo"
    elif delta_y < -pixel_threshold:
        return "acima"

    return "parada"

# Função para decidir a direção da mão
def choose_direction(direction):

    if direction == "direita":
        direita(ser)
    elif direction == "diagonal inferior direita":
        abaixo(ser)
        direita(ser)
    elif direction == "diagonal superior direita":
        acima(ser)
        direita(ser)
    elif direction == "esquerda":
        esquerda(ser)
    elif direction == "diagonal inferior esquerda":
        abaixo(ser)
        esquerda(ser)
    elif direction == "diagonal superior esquerda":
        acima(ser)
        abaixo(ser)
    elif direction == "acima":
        acima(ser)
    elif direction == "abaixo":
        abaixo(ser)

    else:
        parada(ser)
        # Mostra que a mão está parada fazendo o ponto ficar brando
        cv2.circle(frame, (m_coord_x, m_coord_y), 3, (255, 255, 255), 2)

# Carrega o nosso modelo pré treinado
model = YOLO('weight.pt')

# Abre o dispositivo de captura
cap = cv2.VideoCapture(0)

# Tenta criar um objeto para a porta serial
try:
    ser = serial.Serial("COM3", 9600)
    ser.open()
except serial.SerialException:
    pass

# Variáveis para armazenar as coordenadas do ponto central do frame anterior
prev_m_coord_x = 0
prev_m_coord_y = 0

# Loop através dos frames
while cap.isOpened():

    # Extrai o frame e seu status de sucesso na leitura
    read, frame = cap.read()

    if read:

        # Processamento do frame usando o modelo pré treinado
        results = model(source=frame, conf=0.6)

        '''
        Extrai as coordenadas da caixa de detecção de cada mão,
        a informação de qual classe foi detectada e sua confiança
        '''
        boxes = results[0].boxes.cpu().numpy()

        # Métricas para a decisão da melhor detecção
        best_detection = None
        best_score = 0

        # Loop através de todas as detecções na imagem
        for i, box in enumerate(boxes):

            # Detecta se há pelo menos uma mão na imagem
            if box.cls in [0, 1]:

                rect_coord = box.data[0][:4]
                conf = box.data[0][4]
                area = calculateArea(rect_coord)

                # Calcula a localização do ponto central
                m_coord_x = int((box.xyxy[0][2] + box.xyxy[0][0]) / 2)
                m_coord_y = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                # Exibe a área
                cv2.putText(frame, f'Area {i + 1}: {area:.2f}', (int(box.xyxy[0][0]), int(box.xyxy[0][3])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                # Exibe o ponto central
                cv2.circle(frame, (m_coord_x, m_coord_y), 3, (0, 255, 0), 2)

                # Calcula a direção do movimento
                direction = calculate_direction(m_coord_x, m_coord_y, prev_m_coord_x, prev_m_coord_y)

                choose_direction(direction)

                # Atualiza as coordenadas anteriores do ponto central com as atuais
                prev_m_coord_x = m_coord_x
                prev_m_coord_y = m_coord_y

                score = calculateScore(area, conf)

                # Atualiza a melhor detecção se a pontuação for maior
                if score > best_score:
                    best_score = score
                    best_detection = box

        # Verifica se alguma mão foi detectada
        if best_detection is not None:

            # Detecção da classe
            if best_detection.cls == 0:
                fechada(ser)
            elif best_detection.cls == 1:
                aberta(ser)

        else:
            print('No hand detection\n')

        # Plota o resultado do processamento e o exibe
        annotated_frame = results[0].plot(boxes=False)
        cv2.imshow('Camera Feed', annotated_frame)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

    else:
        print('Error while reading camera feed')
        break

releaseConnections(cap, ser)

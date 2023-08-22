import mediapipe as mp
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

# Função da mão acima
def acima(ser):
    try:
        if hand_y >= 0.25 and hand_y < 0.33:
            ser.write(b'41')
        if hand_y >= 0.33 and hand_y < 0.41:
            ser.write(b'42')
        if hand_y >= 0.41 and hand_y <= 0.5:
            ser.write(b'43')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão abaixo
def abaixo(ser):
    try:
        if hand_y > 0.5 and hand_y <= 0.58:
            ser.write(b'51')
        if hand_y > 0.58 and hand_y <= 0.63:
            ser.write(b'52')
        if hand_y > 0.63 and hand_y <= 0.75:
            ser.write(b'53')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão à direita
def direita(ser):
    try:
        if hand_x >= 0.25 and hand_x < 0.3125:
            ser.write(b'21')
        if hand_x >= 0.3125 and hand_x < 0.375:
            ser.write(b'22')
        if hand_x >= 0.375 and hand_x < 0.4375:
            ser.write(b'23')
        if hand_x >= 0.4375 and hand_x <= 0.5:
            ser.write(b'24')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função da mão à esquerda
def esquerda(ser):
    try:
        if hand_x > 0.5 and hand_x <= 0.5625:
            ser.write(b'31')
        if hand_x > 0.5625 and hand_x <= 0.625:
            ser.write(b'32')
        if hand_x > 0.625 and hand_x <= 0.6875:
            ser.write(b'33')
        if hand_x > 0.6875 and hand_x <= 0.75:
            ser.write(b'34')
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função de identificação de setor mais externo
def sectorSelection():
    if hand_x <= 0.5:
        if hand_y <= 0.5:
            # Direita acima
            direita(ser)
            acima(ser)
        elif hand_y > 0.5:
            # Direita abaixo
            direita(ser)
            abaixo(ser)
    elif hand_x > 0.5:
        if hand_y <= 0.5:
            # Esquerda acima
            esquerda(ser)
            acima(ser)
        elif hand_y > 0.5:
            # Esquerda abaixo
            esquerda(ser)
            abaixo(ser)

# Desenha as detecções para que o open-cv possa acessar
mp_drawing = mp.solutions.drawing_utils

# Associa o modelo holístico
mp_holistic = mp.solutions.holistic

# Seleciona o dispositivo de captura
# Cria um overlay no feed da camera com as detecções do modelo holístico
cap = cv2.VideoCapture(0)

# Tenta se conectar a porta serial. Caso não consiga, exibe uma mensagem
try:
    ser = serial.Serial("COMX", 9600, timeout=0.01)
    ser.open()
except serial.SerialException:
    print('Conectando...')

# Inicializa o modelo holístico e dá um nickname
with mp_holistic.Holistic(min_detection_confidence=0.5,
                          min_tracking_confidence=0.5) as holistic:
    # Loop que executa enquanto o dispositivo de captura estiver aberto
    while cap.isOpened():
        # Leu recebe o booleano que indica se a leitura foi feita corretamente
        # Frame recebe o frame do video lido
        read, frame = cap.read()

        # Se não leu corretamente, sai do loop
        if not read:
            break

        # O open cv usa o formato de cor BGR e o mediapipe RGB
        # Pega o frame, converte de BGR para RGB e armazena em 'image'
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Faz as detecções e processa 'image'
        results = holistic.process(image)

        # Após o processamento, converte novamente o frame para BGR
        # Necessário para que o open cv exiba corretamente as cores
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

        # Desenha um retângulo vermelho no frame processado
        cv2.rectangle(image, (rect_top_left_x, rect_top_left_y), (rect_bottom_right_x, rect_bottom_right_y),
                      (0, 0, 255), 3)

        # Detecta se a mão direita foi lida corretamente
        if results.right_hand_landmarks:

            # O resultado do centro da mão direita (landmark[9]) é armazenada em hand_center
            hand_center = results.right_hand_landmarks.landmark[9]
            # Pega a coordenada x e y da mão direita e armazena em hand_x e hand_y
            hand_x = hand_center.x
            hand_y = hand_center.y

            # Define hand_x e hand_y em pixels
            hand_x_pixels = int(hand_x * width)
            hand_y_pixels = int(hand_y * height)

            # Cria uma distância auxiliar para começar a detectar a mão (em pixels)
            safe_distance = 35

            # Define o range em x e em y
            range_left = rect_top_left_x + safe_distance
            range_right = rect_bottom_right_x - safe_distance + 20  # compensação para o formato da mão direita
            range_top = rect_top_left_y + safe_distance
            range_bottom = rect_bottom_right_y - (safe_distance * 2)  # compensação para o formato da mão direita

            # Só desenha as detecções se a mão direita estiver dentro do alcance especificado
            if (hand_x_pixels >= range_left and hand_x_pixels <= range_right) and (
                    hand_y_pixels >= range_top and hand_y_pixels <= range_bottom):
                # Desenha as detecções
                mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                          mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                                          mp_drawing.DrawingSpec(color=(220, 0, 100), thickness=2, circle_radius=2))
                # ind recebe o ponto referente ao dedo indicador
                ind = results.right_hand_landmarks.landmark[8]
                # pol recebe o ponto referente ao dedo polegar
                pol = results.right_hand_landmarks.landmark[4]
                # dist2center recebe o valor usado para definir os limites a partir do centro
                dist2center = 0.08
                # distmin recebe o valor mínimo para se considerar que os dedos se tocaram
                distmin = 0.03
                # Variáveis para os limites esquerdo e direito
                leftL = hand_x - dist2center
                rightL = hand_x + dist2center

                # Detecta se a mão está fechada ou aberta
                if abs(ind.x - pol.x) <= distmin:
                    fechada(ser)
                else:
                    aberta(ser)

                # Posição da mão nos eixos x e y
                sectorSelection()

        # Exibe o frame processado
        cv2.imshow('Camera Feed', image)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

releaseConnections(cap, ser)

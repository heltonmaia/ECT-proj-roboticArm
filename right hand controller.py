import mediapipe as mp
import cv2
import pyautogui as pag

# Configura o tempo entre os comandos automatizados do pyautogui
pag.PAUSE = 0

# Desenha as detecções para que o open-cv possa acessar
mp_drawing = mp.solutions.drawing_utils

# Associa o modelo holístico
mp_holistic = mp.solutions.holistic

# Seleciona o dispositivo de captura
# Cria um overlay no feed da camera com as detecções do modelo holístico
cap = cv2.VideoCapture(0)

# Inicializa o modelo holístico e dá um nickname
with mp_holistic.Holistic(min_detection_confidence=0.5,
                          min_tracking_confidence=0.5) as holistic:
    # Loop que executa enquanto o dispositivo de captura estiver aberto
    while cap.isOpened():
        # Leu recebe o booleano que indica se a leitura foi feita corretamente
        # Frame recebe o frame do video lido
        leu, frame = cap.read()

        # Se não leu corretamente, sai do loop
        if not leu:
            break

        # O open cv usa o formato de cor BGR e o mediapipe RGB
        # Pega o frame, converte de BGR para RGB e armazena em 'image'
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Faz as detecções e processa 'image'
        results = holistic.process(image)

        # Após o processamento, converte novamente o frame para BGR
        # Necessário para que o open cv exiba corretamente as cores
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Desenha as detecções
        mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                  mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                                  mp_drawing.DrawingSpec(color=(220, 0, 100), thickness=2, circle_radius=2))

        # Detecta se a mão direita foi lida corretamente
        if results.right_hand_landmarks:
            # O resultado do centro da mão direita (landmark[9]) é armazenada em hand_center
            hand_center = results.right_hand_landmarks.landmark[9]
            # Pega a coordenada x da mão direita e armazena em hand_x
            hand_x = hand_center.x
            hand_y = hand_center.y
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

            # Detecta se os dois dedos estão nos limites
            if (ind.x >= leftL and ind.x <= rightL) and (pol.x >= leftL and pol.x <= rightL):

                # Detecta se a mão está fechada ou aberta
                if abs(ind.x - pol.x) <= distmin:
                    hand_state = "fechada"
                else:
                    hand_state = "aberta"

                # Posição da mão nos eixos x e y
                if hand_x < 0.5:
                    if hand_y < 0.5:
                        pag.write(f"Direita - Acima ({hand_state})")
                        pag.press('enter')
                    elif hand_y > 0.5:
                        pag.write(f"Direita - Abaixo ({hand_state})")
                        pag.press('enter')
                elif hand_x > 0.5:
                    if hand_y < 0.5:
                        pag.write(f"Esquerda - Acima ({hand_state})")
                        pag.press('enter')
                    elif hand_y > 0.5:
                        pag.write(f"Esquerda - Abaixo ({hand_state})")
                        pag.press('enter')

        # Exibe o frame processado
        cv2.imshow('Camera Feed', image)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

# Libera o acesso e fecha a janela
cap.release()
cv2.destroyAllWindows()

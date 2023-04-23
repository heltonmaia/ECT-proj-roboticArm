import mediapipe as mp
import cv2

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

        # Configuração da cor dos pontos (parâmetro de draw_landmarks)
        # mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2)

        # Configuração da cor dos traços de ligação (parâmetro de draw_landmarks)
        # mp_drawing.DrawingSpec(color=(0, 150, 0), thickness=2, circle_radius=2)

        # Desenha as detecções
        mp_drawing.draw_landmarks(image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                  mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                                  mp_drawing.DrawingSpec(color=(0, 150, 0), thickness=2, circle_radius=2))
        # Exibe o frame processado
        cv2.imshow('Camera Feed', image)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

# Libera o acesso e fecha a janela
cap.release()
cv2.destroyAllWindows()

from ultralytics import YOLO
import cv2
import serial

# Função para liberar conexão de video
def releaseConnections(cap):
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()

# Função para calcular a área da bounding box
def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

# Função para calcular score
def calculateScore():
    return area * conf

# Create a serial object
try:
    ser = serial.Serial("COM8", 9600, timeout=0.01)
    ser.open()
except serial.SerialException:
    print('Conectando...')

# Carrega o nosso modelo pré treinado
model = YOLO('hand-segment-v8.pt')

# Abre o dispositivo de captura
cap = cv2.VideoCapture(0)

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
        for box in boxes:

            # Detecta se há pelo menos uma mão na imagem
            if box.cls in [0, 1]:

                rect_coord = boxes.data[0][:4]
                conf = boxes.data[0][4]
                area = calculateArea(rect_coord)

                score = area

                # Atualiza a melhor detecção se a pontuação for maior
                if score > best_score:
                    best_score = score
                    best_detection = box

        # Verifica se alguma mão foi detectada
        if best_detection is not None:

            # Detecção da classe
            if best_detection.cls == 0:
                ser.write(b'0')
            elif best_detection.cls == 1:
                ser.write(b'1')

        else:
            print('No detection')

        # Plota o resultado do processamento e o exibe
        annotated_frame = results[0].plot(boxes=False)
        cv2.imshow('Camera Feed', annotated_frame)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

    else:
        print('Error while reading camera feed')
        break

releaseConnections(cap)

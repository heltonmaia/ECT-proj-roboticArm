import cv2

# Loop para saber qual o índice do dispositivo de captura
for i in range(10):
    cap = cv2.VideoCapture(i)

    # Se for encontrado pelo método read, indica o índice
    if cap.read()[0]:
        print("Dispositivo de captura encontrado no índice:", i)

    # Libera o acesso
    cap.release()

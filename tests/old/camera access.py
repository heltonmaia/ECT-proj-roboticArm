import cv2
cap = cv2.VideoCapture(0)
# Loop para exibir os frames capturados pela camera numa janela
while True:
    # Leu recebe o booleano que indica se a leitura foi feita corretamente
    # Frame recebe o frame do video lido
    leu, frame = cap.read()

    # Se não leu corretamente, sai do loop
    if not leu:
        break

    # Exibe os frames da camera
    cv2.imshow("Captura de vídeo", frame)

    # Só fecha a janela se o usuário digitar a tecla 't'
    if cv2.waitKey(1) == ord('t'):
        break

# Libera o dispositivo de captura e encerra a execução
cap.release()
cv2.destroyAllWindows()

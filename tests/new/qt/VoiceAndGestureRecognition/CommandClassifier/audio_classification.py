import pyaudio
import struct
import numpy as np
import time
from IPython.display import clear_output
import tensorflow as tf
from .predict import predict
from PyQt6.QtCore import QThread, pyqtSignal

class AudioClassificationThread(QThread):
    audio_update_signal = pyqtSignal(str, float)

    def __init__(self):
        super().__init__()
        self.ThreadActive = False
        self.model = tf.keras.models.load_model('model.keras')
        self.CHUNK = 2048
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        self.THRESHOLD = 350000
        self.RECORD_SECONDS = 2

    def run(self):
        self.ThreadActive = True
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        print("Ouvindo...")
        gravando = False
        audio_data = b""
        while self.ThreadActive:
            data = stream.read(self.CHUNK)
            fmt = f"{self.CHUNK}h"
            data_int = np.array(struct.unpack(fmt, data))

            # Calcula a energia do sinal de áudio
            energia = abs((np.sum(data_int ** 2) / len(data_int)))
            # print(energia)
            # Verifica se a energia é maior que o limiar
            if energia > self.THRESHOLD:
                print("Atividade de voz detectada")
                gravando = True
                start_time = time.time()
            if gravando:
                audio_data += data
            if gravando and (time.time() - start_time) > self.RECORD_SECONDS:
                print("Silêncio")
                command, confidence = predict(self.model, audio_data, self.RECORD_SECONDS, self.CHANNELS, self.RATE)
                self.audio_update_signal.emit(command, confidence)
                gravando = False
                audio_data = b""
                clear_output(wait=True)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def stop(self):
        self.ThreadActive = False
        self.quit()

def posicao_braco(angle_base, angle_garra):
    print(f'angulo da base:{angle_base}')
    print(f'angulo da garra:{angle_garra}')
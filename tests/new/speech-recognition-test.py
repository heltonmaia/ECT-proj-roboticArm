'''
Provê os dados de fala do google
pip install SpeechRecognition

Manipula audios
pip install pyaudio

Opcional (não tenho certeza do que faz)
pip install pyttsx3
'''
import speech_recognition as sr
import os

microfone = sr.Recognizer()

while True:
    with sr.Microphone() as mic:

        microfone.adjust_for_ambient_noise(mic)
        audio = microfone.listen(mic)

        try:
            frase = microfone.recognize_google(audio, language='pt-BR')
            print(f'{frase}\n')
            if "navegador" in frase:
                os.system('start Opera.exe')
        except sr.UnknownValueError():
            microfone = sr.Recognizer()
            continue

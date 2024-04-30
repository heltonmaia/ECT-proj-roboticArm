import speech_recognition as sr
import serial

# Create a serial object
try:
    ser = serial.Serial("COM8", 9600, timeout=0.01)
    ser.open()
except serial.SerialException:
    print('Conectando...')

# Create a recognizer object
r = sr.Recognizer()

# Create a microphone source
mic = sr.Microphone()

# Create a counter
counter = 0

while True:
    # Start listening for speech
    with mic as source:
        audio = r.record(source, duration=5)  # Record for 5 seconds
        print("\nRecording...")

    # Try to recognize the speech
    try:
        print("Recognizing...")
        text = r.recognize_google(audio, language='pt-BR')

        if 'abrir' in text or 'abra' in text or 'open' in text:
            ser.write(b'1')
        if 'fechar' in text or 'feche' in text or 'close' in text:
            ser.write(b'0')
        print(text)

    except:
        print("Sorry, I didn't get that")
        

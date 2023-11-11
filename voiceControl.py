import speech_recognition as sr

# Create a recognizer object
r = sr.Recognizer()

# Create a microphone source
mic = sr.Microphone()

# Create a counter
counter = 0

# Start listening for speech
with mic as source:
    print("Listening...")
    audio = r.record(source, duration=5)  # Record for 3 seconds
    print("Recording...")

# Try to recognize the speech
try:
    print("Recognizing...")
    text = r.recognize_google(audio)
    print("You said: ", text)
except:
    print("Sorry, I didn't get that")
import librosa
from .file_operations import create_temp_audio_file, create_temp_image_file
from .audio_helpers import butter_bandpass_filter
import numpy as np
import cv2
import os
import tensorflow as tf

commands = ["up", "left", "down", "right", "open", "close"]
def predict(model, audio_data, sample_width, channels, rate):
    audio_file = create_temp_audio_file(audio_data, sample_width, channels, rate)
    print(audio_file)
    y_1, sr_1 = librosa.load(audio_file)
    y_1_f = butter_bandpass_filter(y_1, 100, 10000, sr_1, order=5)
    y_1_f_trimmed, _ = librosa.effects.trim(y_1_f, top_db=20)
    amp_y_1 = librosa.stft(y_1_f_trimmed)
    y_1_db = librosa.amplitude_to_db(np.abs(amp_y_1), ref=np.max)

    temp_image_filename = create_temp_image_file(y_1_db)

    image = cv2.imread(temp_image_filename)
    image_tensor = tf.convert_to_tensor(image, dtype=tf.float32) / 255.0
    image_tensor = tf.expand_dims(image_tensor, axis=0)
    prediction = model.predict(image_tensor)
    confidence = np.max(prediction)
    label_pred = np.argmax(prediction, axis=1)
    command = commands[label_pred[0]]
    os.remove(temp_image_filename)
    os.remove(audio_file)
    return command, confidence

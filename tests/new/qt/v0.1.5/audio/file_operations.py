import tempfile
import wave
import matplotlib.pyplot as plt
import librosa


def create_temp_audio_file(audio_data, sample_width, channels, rate):
    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
    try:
        with wave.open(temp_file, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(sample_width)
            wf.setframerate(rate)
            wf.writeframes(audio_data)
        return temp_file.name
    finally:
        temp_file.close()


def create_temp_image_file(y_1_db):
    with tempfile.NamedTemporaryFile(delete=False, suffix='.png') as temp_file:
        temp_filename = temp_file.name
    fig_1, ax_1 = plt.subplots(figsize=(5, 3))
    plt.axis('off')
    librosa.display.specshow(y_1_db, x_axis='time', y_axis='log', ax=ax_1)
    plt.savefig(temp_filename, bbox_inches='tight', pad_inches=0)
    plt.close()

    return temp_filename

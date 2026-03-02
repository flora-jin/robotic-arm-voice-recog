import speech_recognition as sr
import whisper
import pygame
import ollama
import numpy as np
import platform
import os

# Valid Commands
VALID_COMMANDS = ["PLAY_SOUND", "STOP_SOUND"]

# Initialize Pygame
pygame.mixer.init()

# Load Whisper model
print("⏳ Loading Whisper...")
model = whisper.load_model("base")


def listen_for_command():
    """Listen for voice commands and process them"""
    recognizer = sr.Recognizer()

    MICROPHONE_INDEX = 3  # Change if needed

    mic = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=16000)

    print("🎤 Listening...")

    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)

        try:
            audio = recognizer.listen(source, phrase_time_limit=5)

            # Convert audio for Whisper
            audio_data = audio.get_raw_data()
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            result = model.transcribe(audio_np, fp16=False)
            text = result["text"].strip().lower()

            if text:
                print(f"🗣️ Heard: '{text}'")
                return text
            return None

        except Exception as e:
            print(f"Listen error: {e}")
            return None


def ask_ollama(user_text):
    """Classify intent using Ollama"""
    try:
        response = ollama.chat(
            model='llama3',
            messages=[
                {
                    'role': 'system',
                    'content': """
You are an intent classifier.

Valid outputs:
PLAY_SOUND
STOP_SOUND
UNKNOWN

Rules:
- Output ONLY one word.
- No punctuation.
"""
                },
                {'role': 'user', 'content': user_text}
            ],
            options={"temperature": 0}
        )

        raw = response['message']['content']
        command = raw.strip().split()[0].replace(".", "").upper()

        return command if command in VALID_COMMANDS else "UNKNOWN"

    except Exception as e:
        print(f"LLM Error: {e}")
        return "UNKNOWN"


# 🔊 Cross-platform audio playback
def play_audio():
    system = platform.system()
    file_path = "file_path.mp3"  # CHANGE THIS

    try:
        if system == "Darwin":  # macOS
            os.system(f"afplay '{file_path}'")

        elif system == "Linux":
            # Try PulseAudio then ALSA
            os.system(f"paplay '{file_path}' || aplay '{file_path}'")

        else:
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()

        print("🔊 Playing sound...")

    except Exception as e:
        print(f"Playback error: {e}")


def stop_audio():
    """Stop audio (works if pygame fallback is used)"""
    try:
        pygame.mixer.music.stop()
        print("⏹️ Sound stopped.")
    except Exception:
        pass


def main():
    """Main loop"""
    print("🚀 Voice Command Node Ready")

    while True:
        user_command = listen_for_command()

        if not user_command:
            continue

        command_id = ask_ollama(user_command)

        if command_id == "PLAY_SOUND":
            print("Command: PLAY_SOUND")
            play_audio()

        elif command_id == "STOP_SOUND":
            print("Command: STOP_SOUND")
            stop_audio()

        else:
            print("Intent unclear")


if __name__ == "__main__":
    main()
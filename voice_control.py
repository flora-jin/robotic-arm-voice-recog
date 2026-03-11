import speech_recognition as sr
import whisper
import pygame
import ollama
import numpy as np
import platform
import os
import threading
import subprocess

# fastapi things
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# ------------------------------------------------

audio_options=["mozart.mp3", "white_noise.mp3", "news.mp3"]
current_sound = audio_options[0]

# FastAPI app
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins. For production, specify your frontend's URL.
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods (GET, POST, etc.)
    allow_headers=["*"],  # Allows all headers
)

@app.get("/")
def root():
    return {"status": "Voice node running"}

@app.get("/get_sounds")
def get_sounds():
    return audio_options

# usage:
# POST http://localhost:8000/change_sound?audio_file="mozart.mp3"
@app.post("/change_sound")
def change_sound(audio_file: str):
    global current_sound

    if audio_file not in audio_options:
        return {
            "message": "Sound not available"
        }

    current_sound = audio_file

    return {
        "message": "Sound updated",
        "current_sound": current_sound
    }

@app.get("/health")
def health():
    return {"ok": True}

def run_api():
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

# ------------------------------------------------

# Valid Commands
VALID_COMMANDS = ["PLAY_MOZART", "PLAY_NEWS", "PLAY_WHITE_NOISE", "STOP_SOUND"]

# Initialize Pygame
pygame.mixer.init()

# Load Whisper model
print("⏳ Loading Whisper...")
model = whisper.load_model("base")

# Initialize audio process
audio_process = None

def listen_for_command():
    """Listen for voice commands and process them"""
    recognizer = sr.Recognizer()

    MICROPHONE_INDEX = 1  # Change if needed

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
PLAY_MOZART
PLAY_NEWS
PLAY_WHITE_NOISE
STOP_SOUND
UNKNOWN

Rules:
- Output ONLY one word.
- No punctuation.
- If the user asks for Mozart or music, output PLAY_MOZART.
- If the user asks for news, output PLAY_NEWS.
- If the user asks for white noise, output PLAY_WHITE_NOISE.
- If the user asks to stop, output STOP_SOUND.
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
    global audio_process
    global current_sound

    system = platform.system()
    file_path = current_sound

    try:
        if system == "Darwin":  # macOS
            audio_process = subprocess.Popen(["afplay", file_path])

        elif system == "Linux":
            audio_process = subprocess.Popen(["paplay", file_path])

        else:
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()

        print("🔊 Playing sound...")

    except Exception as e:
        print(f"Playback error: {e}")


def stop_audio():
    """Stop audio (works if pygame fallback is used)"""
    global audio_process

    try:
        if audio_process:
            audio_process.terminate()
            audio_process = None

        pygame.mixer.music.stop()
        print("⏹️ Sound stopped.")

    except Exception as e:
        print(f"Stope error: {e}")

def main():
    """Main loop"""
    print("🚀 Voice Command Node Ready")

    while True:
        user_command = listen_for_command()

        if not user_command:
            continue

        command_id = ask_ollama(user_command)

        global current_sound
        if command_id == "PLAY_MOZART":
            print("Command: PLAY_MOZART")
            current_sound = "mozart.mp3"
            threading.Thread(target=play_audio, daemon=True).start()

        elif command_id == "PLAY_NEWS":
            print("Command: PLAY_NEWS")
            current_sound = "news.mp3"
            threading.Thread(target=play_audio, daemon=True).start()

        elif command_id == "PLAY_WHITE_NOISE":
            print("Command: PLAY_WHITE_NOISE")
            current_sound = "white_noise.mp3"
            threading.Thread(target=play_audio, daemon=True).start()

        elif command_id == "STOP_SOUND":
            print("Command: STOP_SOUND")
            stop_audio()

        else:
            print("Intent unclear")


if __name__ == "__main__":
    # Start FastAPI in background thread
    api_thread = threading.Thread(target=run_api, daemon=True)
    api_thread.start()

    main()
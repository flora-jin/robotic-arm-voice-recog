import speech_recognition as sr
import whisper
import numpy as np
import threading

# fastapi things
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from intent_parser import ask_ollama
from soundscape import get_sounds, set_current_sound, get_current_sound, execute_command, stop_audio
from telepathy import is_telepathy_command, execute_telepathy_command
from ventriloquism import is_ventriloquism_command, execute_ventriloquism_command
import pygame
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
def api_get_sounds():
    return get_sounds()

# usage:
# POST http://localhost:8000/change_sound?audio_file="mozart.mp3"
@app.post("/change_sound")
def change_sound(audio_file: str):
    if set_current_sound(audio_file):
        return {
            "message": "Sound updated",
            "current_sound": get_current_sound()
        }
    
    return {
        "message": "Sound not available"
    }

@app.get("/health")
def health():
    return {"ok": True}

def run_api():
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

# ------------------------------------------------

# Load Whisper model
print("⏳ Loading Whisper...")
model = whisper.load_model("base")

recognizer = sr.Recognizer()
# MICROPHONE_INDEX = 4  # Change if needed
mic = sr.Microphone()

def listen_for_command(source):
    """Listen for voice commands and process them"""
    print("🎤 Listening...")

    try:
        audio = recognizer.listen(source, phrase_time_limit=5)

        # Convert audio for Whisper
        audio_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
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


def main():
    """Main loop"""
    print("🚀 Voice Command Node Ready")

    # Gracefully attempt to init ROS node if running in Linux environment
    try:
        import rospy
        rospy.init_node('voice_control_node', anonymous=True)
    except ImportError:
        pass # Already handled print in ventriloquism.py

    print("🔧 Adjusting for ambient noise...")
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)
        
        active_mode = False

        while True:
            user_command = listen_for_command(source)

            if not user_command:
                continue
                
            # Voice Activity Trigger Logic
            if "hey robot arm" in user_command:
                # Optional: Handle case where wakeword and command are in the same breath (e.g. "hey robot arm check window")
                active_mode = True
                print("🟢 Woken up! Ready to process commands.")
                # Play confirmation beep
                stop_audio(verbose=False)
                try:
                    pygame.mixer.music.load("telepathy_sounds/confirm.mp3")
                    pygame.mixer.music.play()
                except Exception as e:
                    pass
                continue
                
            if "end task" in user_command:
                if active_mode:
                    active_mode = False
                    print("🔴 Going to sleep. Say 'Hey robot arm' to wake me.")
                    # Play confirmation sleep beep
                    stop_audio(verbose=False)
                    try:
                        pygame.mixer.music.load("telepathy_sounds/confirm.mp3")
                        pygame.mixer.music.play()
                    except Exception as e:
                        pass
                continue
                
            # If not active, ignore the ambient noise
            if not active_mode:
                # print("💤 Ignored ambient noise while asleep.")
                continue

            # Only reach here if active_mode is True and no sleepword was triggered
            command_id = ask_ollama(user_command)

            if is_telepathy_command(command_id):
                execute_telepathy_command(command_id, source, recognizer, model)
            elif is_ventriloquism_command(command_id):
                execute_ventriloquism_command(command_id, source, recognizer, model, user_command)
            else:
                execute_command(command_id)


if __name__ == "__main__":
    try:
        # Start FastAPI in background thread
        api_thread = threading.Thread(target=run_api, daemon=True)
        api_thread.start()

        main()
    except KeyboardInterrupt:
        pass
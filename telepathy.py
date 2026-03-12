import pygame
import threading
import time
import numpy as np
from soundscape import stop_audio

telepathy_commands = ["SET_TIMER", "SEND_REMINDER", "WISH_BIRTHDAY"]

def is_telepathy_command(command_id):
    return command_id in telepathy_commands

# 🔊 Cross-platform audio playback specifically for Telepathy multi-track flow
def play_telepathy_sequence(track_name, source, recognizer, model):
    """Wait for confirm, then play confirm.mp3 followed by the specific track_name"""
    
    # Prompt the user to confirm first
    print("\n💡 Telepathy: Awaiting verbal 'confirm' from user...")
    
    # Listen specifically for the word "confirm"
    confirmed = False
    
    # Briefly adjust for ambient noise for better accuracy on short words
    recognizer.adjust_for_ambient_noise(source, duration=0.5)

    # We loop until we hear them explicitly say confirm (or cancel)
    while not confirmed:
        try:
            audio = recognizer.listen(source, phrase_time_limit=7)
            audio_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            result = model.transcribe(audio_np, fp16=False)
            text = result["text"].strip().lower()

            if text:
                print(f"🗣️ Heard (Telepathy): '{text}'")
                if "confirm" in text or "yes" in text:
                    confirmed = True
                    print("✅ User confirmed!")
                elif "cancel" in text or "no" in text or "stop" in text:
                    print("❌ User cancelled telepathy action.")
                    return # Exit without playing anything

        except Exception as e:
            print(f"Telepathy listen error: {e}")

    # Stop any currently playing audio quietly so they don't overlap
    stop_audio(verbose=False)

    try:
        # 1. Play Confirmation Sound
        print("🔊 Telepathy: Playing telepathy_sounds/confirm.mp3...")
        pygame.mixer.music.load("telepathy_sounds/confirm.mp3")
        pygame.mixer.music.play()
        
        # Wait until confirm.mp3 finishes playing
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
            
        # 2. Play the requested sound
        print(f"🔊 Telepathy: Playing telepathy_sounds/{track_name}...")
        pygame.mixer.music.load(f"telepathy_sounds/{track_name}")
        pygame.mixer.music.play()

    except Exception as e:
        print(f"Telepathy playback error: {e}")

def execute_telepathy_command(command_id, source, recognizer, model):
    """Execute the action mapped to the telepathy command_id."""
    
    if command_id == "SET_TIMER":
        play_telepathy_sequence("timer.mp3", source, recognizer, model)

    elif command_id == "SEND_REMINDER":
        play_telepathy_sequence("reminder.mp3", source, recognizer, model)

    elif command_id == "WISH_BIRTHDAY":
        play_telepathy_sequence("birthday.mp3", source, recognizer, model)

    else:
        print("Telepathy intent unclear")

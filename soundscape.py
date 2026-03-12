import pygame
import threading

# Initialize Pygame globally in this module to be ready for audio actions
pygame.mixer.init()

audio_options = ["mozart.mp3", "white_noise.mp3", "news.mp3"]
current_sound = audio_options[0]

def get_sounds():
    return audio_options

def set_current_sound(sound):
    global current_sound
    if sound in audio_options:
        current_sound = sound
        return True
    return False

def get_current_sound():
    global current_sound
    return current_sound

def stop_audio(verbose=True):
    """Stop audio"""
    try:
        if pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
            if verbose:
                print("⏹️ Sound stopped.")
    except Exception as e:
        if verbose:
            print(f"Stop error: {e}")

# 🔊 Cross-platform audio playback
def play_audio():
    global current_sound

    # Stop any currently playing audio quietly so they don't overlap
    stop_audio(verbose=False)

    file_path = current_sound

    try:
        pygame.mixer.music.load(f"sounds/{file_path}")
        pygame.mixer.music.play()
        print(f"🔊 Playing sounds/{file_path}...")

    except Exception as e:
        print(f"Playback error: {e}")

def execute_command(command_id):
    """Execute the action mapped to the command_id."""
    global current_sound
    
    if command_id == "PLAY_MOZART":
        current_sound = "mozart.mp3"
        threading.Thread(target=play_audio, daemon=True).start()

    elif command_id == "PLAY_NEWS":
        current_sound = "news.mp3"
        threading.Thread(target=play_audio, daemon=True).start()

    elif command_id == "PLAY_WHITE_NOISE":
        current_sound = "white_noise.mp3"
        threading.Thread(target=play_audio, daemon=True).start()

    elif command_id == "STOP_SOUND":
        stop_audio(verbose=True)

    else:
        print("Intent unclear")

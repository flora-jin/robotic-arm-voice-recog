import pygame
import threading
import time
import numpy as np
from soundscape import stop_audio

# Graceful ROS imports for testing on non-ROS Linux/Mac machines
try:
    import rospy
    from std_msgs.msg import String
    ros_available = True
    print("🤖 ROS detected. Ventriloquism will publish to /ventriloquism_commands")
    
    # Lazy Initialization of the Publisher
    ventriloquism_pub = rospy.Publisher('/ventriloquism_commands', String, queue_size=10)
    
except ImportError:
    ros_available = False
    print("⚠️ ROS not detected. Ventriloquism will run locally without publishing.")
    
# Dictionary to hold the mapping from location to action (sound)
# Default empty dict to start
location_to_action = {}

ventriloquism_commands = [
    "ASSIGN_WEATHER_REPORT", "ASSIGN_READ_ALOUD", "ASSIGN_LAST_UPDATED",
    "CHECK_WINDOW", "CHECK_WHITEBOARD", "CHECK_PLANT",
    "CHECK_WEATHER_REPORT", "CHECK_READ_ALOUD", "CHECK_LAST_UPDATED"
]

def is_ventriloquism_command(command_id):
    return command_id in ventriloquism_commands

def wait_for_confirm(source, recognizer, model):
    """Wait for the user to verbally confirm."""
    print("\n🗣️ Ventriloquism: Awaiting verbal 'confirm' from user...")
    confirmed = False
    
    # Briefly adjust for ambient noise for better accuracy on short words
    recognizer.adjust_for_ambient_noise(source, duration=0.5)

    while not confirmed:
        try:
            audio = recognizer.listen(source, phrase_time_limit=7)
            audio_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            result = model.transcribe(audio_np, fp16=False)
            text = result["text"].strip().lower()

            if text:
                print(f"🗣️ Heard (Ventriloquism): '{text}'")
                if "confirm" in text or "yes" in text:
                    confirmed = True
                    print("✅ User confirmed!")
                    return True
                elif "cancel" in text or "no" in text or "stop" in text:
                    print("❌ User cancelled ventriloquism action.")
                    return False

        except Exception as e:
            print(f"Ventriloquism listen error: {e}")
            
    return False

def play_sound(track_name):
    """Play a specific track for check action"""
    stop_audio(verbose=False)
    try:
        print(f"🔊 Ventriloquism: Playing ventriloquism_sounds/{track_name}...")
        pygame.mixer.music.load(f"ventriloquism_sounds/{track_name}")
        pygame.mixer.music.play()
    except Exception as e:
        print(f"Ventriloquism playback error: {e}")


def execute_ventriloquism_command(command_id, source, recognizer, model, original_text):
    """Execute the action mapped to the ventriloquism command_id."""
    
    # Handle Check commands
    if command_id.startswith("CHECK_"):
        target = command_id.split("CHECK_")[1] # e.g. "WINDOW" or "WEATHER_REPORT"
        
        target_lower = target.lower()
        locations = ["window", "whiteboard", "plant"]
        
        action = None
        
        # Scenario A: User asked to check a location
        if target_lower in locations:
            location = target_lower
            if location in location_to_action:
                action = location_to_action[location]
                print(f"🔍 Ventriloquism: Checking {location}. Playing assigned action: {action}")
            else:
                print(f"🔍 Ventriloquism: Checking {location}. No action assigned yet.")
                return
                
        # Scenario B: User asked to check an action directly
        else:
            action = target
            # Optional: ensure this action is actually assigned anywhere before playing
            if action not in location_to_action.values():
                print(f"🔍 Ventriloquism: Checking {action}. This action is not currently assigned to any location.")
                return
            else:
                print(f"🔍 Ventriloquism: Checking {action}. It is currently assigned to a location.")
                for loc, act in location_to_action.items():
                    if act == action:
                        location = loc
                        break

        # Map action back to sound file
        if action == "WEATHER_REPORT":
            play_sound("weather.mp3") 
        elif action == "READ_ALOUD":
            play_sound(f"read_{location}.mp3")
        elif action == "LAST_UPDATED":
            play_sound(f"update_{location}.mp3")
            
        if ros_available:
            msg = f"check,{action}"
            ventriloquism_pub.publish(msg)
            print(f"📡 Published ROS state: {msg}")
            
    # Handle Assign commands
    elif command_id.startswith("ASSIGN_"):
        # We need to figure out which location they meant from the original text if it's not strictly structured
        # The Ollama prompt will return "ASSIGN_WEATHER_REPORT" but the text might be "Assign weather report to the plant"
        
        action_parts = command_id.split("_")[1:]
        action_name = "_".join(action_parts) # WEATHER_REPORT

        # Find location in the text
        text_lower = original_text.lower()
        target_location = None
        if "window" in text_lower:
            target_location = "window"
        elif "whiteboard" in text_lower:
            target_location = "whiteboard"
        elif "plant" in text_lower:
            target_location = "plant"
            
        if not target_location:
            print("❌ Ventriloquism: Could not determine location to assign to. Please specify window, whiteboard, or plant.")
            return

        print(f"🔄 Ventriloquism: Found assignment request for {action_name} to {target_location}.")
        
        # Wait for confirm
        if wait_for_confirm(source, recognizer, model):
            location_to_action[target_location] = action_name
            print(f"✅ Ventriloquism: Successfully assigned {action_name} to {target_location}!")
            print(f"Current dict state: {location_to_action}")
            
            if ros_available:
                msg = f"assign,{action_name}"
                ventriloquism_pub.publish(msg)
                print(f"📡 Published ROS state: {msg}")
            
            # Play confirm.mp3 from telepathy sounds folder as feedback
            stop_audio(verbose=False)
            try:
                pygame.mixer.music.load("telepathy_sounds/confirm.mp3")
                pygame.mixer.music.play()
            except Exception as e:
                pass

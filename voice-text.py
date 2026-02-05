#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import whisper
import numpy as np
import ssl
import certifi

# SSL Fix
ssl._create_default_https_context = lambda: ssl.create_default_context(cafile=certifi.where())

def speech_node():
    # 1. Init ROS Node
    rospy.init_node('speech_transcriber', anonymous=True)
    pub = rospy.Publisher('/voice/raw_text', String, queue_size=10)
    
    # 2. Load Whisper
    print("⏳ Loading Whisper...")
    model = whisper.load_model("base")
    recognizer = sr.Recognizer()

    # Index for selecting desired microphone
    MICROPHONE_INDEX = 1  # Change this to the index found

    # Initialize Mic with 16kHz Sample Rate
    # Whisper requires 16000Hz. If we don't set this, it defaults to 44100/48000 which would make audio sound slow motion
    mic = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=16000)

    print("🎤 Speech Node Ready. Listening...")

    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)
        
        while not rospy.is_shutdown():
            try:
                # Listen
                print("Listening for phrase...")
                audio = recognizer.listen(source, phrase_time_limit=5)
                
                # Transcribe
                audio_data = audio.get_raw_data()
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                result = model.transcribe(audio_np, fp16=False)
                text = result["text"].strip().lower()

                if text:
                    print(f"🗣️ Heard: '{text}'")
                    # Publish the RAW text (e.g., "go back home please")
                    pub.publish(text)

            except sr.WaitTimeoutError:
                pass
            except Exception as e:
                print(f"Error: {e}")

if __name__ == '__main__':
    speech_node()
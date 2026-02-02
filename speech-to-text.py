import ssl
import certifi
import speech_recognition as sr
import whisper
import numpy as np
import time

# 1. SSL Context Fix
ssl._create_default_https_context = lambda: ssl.create_default_context(cafile=certifi.where())

# 2. Load Model
print("⏳ Loading Whisper Model...")
model = whisper.load_model("base")
recognizer = sr.Recognizer()

# --- CONFIGURATION ---
MICROPHONE_INDEX = 1  # Change this to the index you found!
# ---------------------

# 3. Initialize Mic with 16kHz Sample Rate
# Whisper requires 16000Hz. If we don't set this, it defaults to 44100/48000
# and the audio sounds like slow-motion to the model.
mic = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=16000)

print("🎤 Listening...")

last_cmd_time = 0

with mic as source:
    # Quick ambient noise adjustment
    recognizer.adjust_for_ambient_noise(source, duration=1)
    
    while True:
        try:
            # Capture audio
            print("Listening for phrase...")
            audio = recognizer.listen(source, phrase_time_limit=3)

            # Convert raw bytes (16-bit int) to NumPy float32
            # because we set sample_rate=16000 above, this data is already 16k
            audio_data = audio.get_raw_data()
            
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            # Transcribe
            result = model.transcribe(audio_np, fp16=False)
            text = result["text"].strip().lower()

            if not text:
                continue

            print(f"💬 Recognized: '{text}'")

            # Simple debounce logic
            if time.time() - last_cmd_time < 1.0:
                continue

            if "stop" in text:
                print("🛑 STOP COMMAND RECOGNIZED")
                last_cmd_time = time.time()
                # You might want to break the loop here if you actually want to quit
                # break 

        except sr.WaitTimeoutError:
            pass # No speech detected, ignore
        except Exception as e:
            print(f"Error: {e}")
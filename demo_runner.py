#!/usr/bin/env python3
"""
demo_runner.py — walk through every command with no microphone attached.

    python demo_runner.py                    # type utterances yourself
    python demo_runner.py --script full      # canned walkthrough, Enter to advance
    python demo_runner.py --script full --auto --fast   # hands-free for recording

You can type a plain sentence ("play some mozart") or the command ID itself
("PLAY_MOZART") — typing the ID skips intent classification entirely, which is
handy when you need a take to come out the same way twice.

Meta commands at the prompt: /state  /wake  /sleep  /help  /quit
"""

import argparse
import sys
import time
import types

import mock_io

mock_io.install()  # must happen before the project modules are imported

import numpy as np                     # noqa: E402
import speech_recognition as sr        # noqa: E402
import whisper                         # noqa: E402

import soundscape                      # noqa: E402
import telepathy                       # noqa: E402
import ventriloquism                   # noqa: E402
from intent_parser import VALID_COMMANDS, ask_ollama          # noqa: E402
from intent_similarity import is_similar_command              # noqa: E402

WAKE_PHRASE = "wake up wake up"
STOP_PHRASE = "stop task"

recognizer = sr.Recognizer()
recognizer.dynamic_energy_threshold = True
recognizer.pause_threshold = 1.5
recognizer.non_speaking_duration = 0.5
recognizer.phrase_threshold = 0.3

mic = sr.Microphone()
model = whisper.load_model("base")


# --------------------------------------------------------------------------
# Offline intent classification (same rules as the Ollama system prompt)
# --------------------------------------------------------------------------

def _pad(text):
    cleaned = "".join(c if c.isalnum() else " " for c in text.lower())
    return " " + " ".join(cleaned.split()) + " "


def keyword_classify(text):
    t = _pad(text)

    def has(*words):
        return any(f" {w} " in t for w in words)

    if has("assign", "assigned", "map", "put"):
        if has("weather"):
            return "ASSIGN_WEATHER_REPORT"
        if has("read", "reading", "aloud"):
            return "ASSIGN_READ_ALOUD"
        if has("updated", "update", "last"):
            return "ASSIGN_LAST_UPDATED"

    if has("check", "checking", "look"):
        if has("whiteboard"):
            return "CHECK_WHITEBOARD"
        if has("window"):
            return "CHECK_WINDOW"
        if has("plant"):
            return "CHECK_PLANT"
        if has("weather"):
            return "CHECK_WEATHER_REPORT"
        if has("read", "reading", "aloud"):
            return "CHECK_READ_ALOUD"
        if has("updated", "update"):
            return "CHECK_LAST_UPDATED"

    if has("timer"):
        return "SET_TIMER"
    if has("reminder", "remind", "meeting"):
        return "SEND_REMINDER"
    if has("birthday"):
        return "WISH_BIRTHDAY"
    if has("stop", "quiet", "silence", "off", "enough", "mute"):
        return "STOP_SOUND"
    if has("mozart", "classical", "music", "piano"):
        return "PLAY_MOZART"
    if has("news", "headlines", "radio"):
        return "PLAY_NEWS"
    if has("static") or " white noise " in t or has("noise"):
        return "PLAY_WHITE_NOISE"
    return "UNKNOWN"


def classify(text, use_llm):
    typed_id = text.strip().upper().replace(" ", "_")
    if typed_id in VALID_COMMANDS:
        return typed_id, "typed directly"

    if use_llm and mock_io.OLLAMA_AVAILABLE:
        command = ask_ollama(text)
        if command != "UNKNOWN":
            return command, "ollama"

    return keyword_classify(text), "keyword fallback"


# --------------------------------------------------------------------------
# Same listening pipeline as voice_control.py, only the hardware is faked
# --------------------------------------------------------------------------

def listen_for_command(source):
    try:
        audio = recognizer.listen(source, timeout=3, phrase_time_limit=None)
        audio_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
        audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
        result = model.transcribe(audio_np, fp16=False)
        text = result["text"].strip().lower()
        if text:
            print(f"🗣️  Heard: '{text}'")
            return text
        return None
    except sr.WaitTimeoutError:
        return None


def play_confirmation_beep():
    soundscape.stop_audio(verbose=False)
    import pygame
    try:
        pygame.mixer.music.load("telepathy_sounds/confirm.mp3")
        pygame.mixer.music.play()
    except Exception:
        soundscape.play_repeat_prompt()


# --------------------------------------------------------------------------
# Canned walkthroughs
# --------------------------------------------------------------------------

SCRIPTS = {
    "soundscape": [
        "wake up wake up",
        "play some mozart",
        "can you put the news on",
        "give me some white noise",
        "stop the sound",
    ],
    "telepathy": [
        "wake up wake up",
        "set a timer for me",
        "confirm",
        "send a meeting reminder",
        "confirm",
        "wish her a happy birthday",
        "cancel",
    ],
    "ventriloquism": [
        "wake up wake up",
        "assign the weather report to the window",
        "confirm",
        "assign read aloud to the plant",
        "confirm",
        "check the window",
        "check the plant",
        "check the whiteboard",
        "assign last updated to the whiteboard",
        "confirm",
        "check the whiteboard",
        "check the weather report",
    ],
}

SCRIPTS["full"] = (
    SCRIPTS["soundscape"]
    + SCRIPTS["telepathy"][1:]
    + SCRIPTS["ventriloquism"][1:]
    + ["stop task"]
)


# --------------------------------------------------------------------------
# Meta commands
# --------------------------------------------------------------------------

HELP = """
Meta commands
  /state   show the current sound and the location -> action map
  /wake    force the node awake
  /sleep   force the node asleep
  /help    this text
  /quit    exit

Anything else is treated as an utterance. Type a command ID
(e.g. CHECK_PLANT) to skip intent classification.
"""


def show_state():
    print("\n──────── state ────────")
    print(f"  current sound : {soundscape.get_current_sound()}")
    mapping = ventriloquism.location_to_action
    if mapping:
        for location, action in mapping.items():
            print(f"  {location:<11}: {action}")
    else:
        print("  (no locations assigned yet)")
    print("───────────────────────")


# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--script", choices=sorted(SCRIPTS), help="run a canned walkthrough")
    parser.add_argument("--auto", action="store_true",
                        help="advance the script on a timer instead of waiting for Enter")
    parser.add_argument("--delay", type=float, default=2.0, help="seconds between scripted lines")
    parser.add_argument("--fast", action="store_true",
                        help="shorten simulated track lengths and the 3s robot-motion pause")
    parser.add_argument("--awake", action="store_true", help="start already woken up")
    parser.add_argument("--llm", action="store_true",
                        help="classify with Ollama instead of the offline keyword rules")
    parser.add_argument("--api", action="store_true",
                        help="also serve the FastAPI endpoints on :8000")
    args = parser.parse_args()

    if args.fast:
        mock_io.TIME_SCALE = 0.35
        # ventriloquism sleeps 3s before CHECK audio to cover the arm's motion
        ventriloquism.time = types.SimpleNamespace(sleep=lambda s: time.sleep(min(s, 0.4)))

    if args.script:
        mock_io.PROMPTER.load_script(SCRIPTS[args.script])
        mock_io.PROMPTER.step = not args.auto
        mock_io.PROMPTER.delay = args.delay

    if args.api:
        try:
            import threading
            import uvicorn
            from voice_control import app
            threading.Thread(
                target=lambda: uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning"),
                daemon=True,
            ).start()
            print("🌐 FastAPI on http://localhost:8000")
        except Exception as exc:
            print(f"⚠️  API not started: {exc}")

    use_llm = args.llm and mock_io.OLLAMA_AVAILABLE
    if args.llm and not use_llm:
        print("⚠️  Ollama unavailable — falling back to keyword classification.")

    print("🚀 Voice Command Node Ready (mock mode)")
    print(HELP)

    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)
        active_mode = args.awake
        if active_mode:
            print("🟢 Starting awake.")

        while True:
            user_command = listen_for_command(source)
            if not user_command:
                continue

            if user_command.startswith("/"):
                meta = user_command.strip().lower()
                if meta == "/state":
                    show_state()
                elif meta == "/wake":
                    active_mode = True
                    print("🟢 Forced awake.")
                elif meta == "/sleep":
                    active_mode = False
                    print("🔴 Forced asleep.")
                else:
                    print(HELP)
                continue

            if is_similar_command(user_command, WAKE_PHRASE):
                active_mode = True
                print("🟢 Woken up! Ready to process commands.")
                play_confirmation_beep()
                continue  # <- the original falls through here and errors on the wake phrase

            if is_similar_command(user_command, STOP_PHRASE):
                if active_mode:
                    active_mode = False
                    print("🔴 Going to sleep. Say 'wake up wake up' to wake me.")
                    play_confirmation_beep()
                continue

            if not active_mode:
                print("💤 Asleep — say 'wake up wake up' first.")
                continue

            command_id, how = classify(user_command, use_llm)
            print(f"🧠 Intent: {command_id}   [{how}]")

            if telepathy.is_telepathy_command(command_id):
                telepathy.execute_telepathy_command(command_id, source, recognizer, model)
            elif ventriloquism.is_ventriloquism_command(command_id):
                ventriloquism.execute_ventriloquism_command(
                    command_id, source, recognizer, model, user_command
                )
            else:
                soundscape.execute_command(command_id)
                # soundscape plays on a daemon thread; let it print before we
                # draw the next prompt, so the transcript stays readable on video
                time.sleep(0.3)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, mock_io.DemoExit):
        print("\n👋 Demo ended.")
        sys.exit(0)

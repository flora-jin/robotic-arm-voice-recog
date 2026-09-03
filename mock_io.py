"""
mock_io.py — microphone-free, speaker-free stand-ins for the voice node.

Call install() BEFORE importing soundscape / telepathy / ventriloquism /
voice_control. It swaps fake modules into sys.modules for:

    pygame              -> prints what would play, and fakes get_busy() timing
    speech_recognition  -> reads typed lines instead of listening to a mic
    whisper             -> returns whatever was typed, no model download
    numpy               -> tiny stub, only if real numpy is missing
    ollama              -> harmless stub, only if real ollama is missing

None of the original project files are modified.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import time
import types

# --------------------------------------------------------------------------
# Tunables
# --------------------------------------------------------------------------

TIME_SCALE = 1.0          # multiplier on all simulated track lengths
DEFAULT_SECONDS = 2.5     # simulated length of an unknown track
TRACK_SECONDS = {         # simulated length per filename
    "confirm.mp3": 1.0,
    "confirm_question.mp3": 2.0,
    "error.mp3": 1.5,
}

QUIT_WORDS = {"/quit", "/exit", "quit", "exit"}

OLLAMA_AVAILABLE = False  # set by install()


class DemoExit(BaseException):
    """Raised from a mocked listen() to unwind out of the demo.

    Inherits from BaseException on purpose: telepathy.py and ventriloquism.py
    wrap their confirmation loops in `except Exception`, so a normal exception
    would be swallowed and the loop would spin forever.
    """


# --------------------------------------------------------------------------
# Shared state between the fake recognizer and the fake whisper model
# --------------------------------------------------------------------------

_state = {"last_text": ""}


class _Prompter:
    """Where mocked utterances come from: the keyboard, or a canned script."""

    def __init__(self):
        self.script: list[str] = []
        self.step = True      # wait for Enter between scripted lines
        self.delay = 2.0      # ...or sleep this long when step is False

    def load_script(self, lines):
        self.script = list(lines)

    def get(self, label="🎤 say>"):
        if self.script:
            line = self.script.pop(0)
            if self.step:
                try:
                    input(f"\n{label} (next: press Enter) ")
                except EOFError:
                    raise DemoExit
                print(f"⌨️  {line}")
            else:
                time.sleep(self.delay)
                print(f"\n{label} {line}")
            return line

        try:
            line = input(f"\n{label} ")
        except EOFError:
            raise DemoExit
        if line.strip().lower() in QUIT_WORDS:
            raise DemoExit
        return line


PROMPTER = _Prompter()


# --------------------------------------------------------------------------
# Fake pygame
# --------------------------------------------------------------------------

class _MockMusic:
    def __init__(self):
        self._loaded = None
        self._ends_at = 0.0

    def load(self, path, *args, **kwargs):
        path = str(path)
        self._loaded = path
        self._ends_at = 0.0
        tag = "on disk" if os.path.exists(path) else "not on disk — mock only"
        print(f"   🎚️  [AUDIO] load  {path}   ({tag})")

    def play(self, *args, **kwargs):
        if self._loaded is None:
            raise RuntimeError("mock pygame: play() called before load()")
        secs = TRACK_SECONDS.get(os.path.basename(self._loaded), DEFAULT_SECONDS)
        secs *= TIME_SCALE
        self._ends_at = time.monotonic() + secs
        print(f"   ▶️  [AUDIO] play  {self._loaded}   (~{secs:.1f}s simulated)")

    def stop(self, *args, **kwargs):
        if self.get_busy():
            print(f"   ⏹️  [AUDIO] stop  {self._loaded}")
        self._ends_at = 0.0

    def get_busy(self):
        return time.monotonic() < self._ends_at

    def unload(self, *args, **kwargs):
        self._loaded = None

    def set_volume(self, *args, **kwargs):
        pass

    def get_volume(self):
        return 1.0

    def pause(self, *args, **kwargs):
        pass

    def unpause(self, *args, **kwargs):
        pass


def _build_pygame():
    mixer = types.ModuleType("pygame.mixer")
    mixer.music = _MockMusic()
    mixer._inited = False

    def init(*args, **kwargs):
        mixer._inited = True
        print("🔈 [MOCK] pygame.mixer.init() — no audio device needed")

    def quit_(*args, **kwargs):
        mixer._inited = False

    def get_init():
        return (44100, -16, 2) if mixer._inited else None

    mixer.init = init
    mixer.quit = quit_
    mixer.get_init = get_init

    pygame = types.ModuleType("pygame")
    pygame.mixer = mixer
    pygame.error = RuntimeError

    def pg_init(*args, **kwargs):
        return (0, 0)

    pygame.init = pg_init
    pygame.__mock__ = True
    return pygame, mixer


# --------------------------------------------------------------------------
# Fake speech_recognition
# --------------------------------------------------------------------------

class _MockAudioData:
    """Stands in for sr.AudioData. The text rides along in _state."""

    def __init__(self, text):
        self.text = text

    def get_raw_data(self, convert_rate=None, convert_width=None):
        # Dummy PCM: length must be a multiple of 2 for np.frombuffer(int16).
        return b"\x00\x00" * 160


class _MockRecognizer:
    def __init__(self):
        self.energy_threshold = 300
        self.dynamic_energy_threshold = True
        self.pause_threshold = 0.8
        self.non_speaking_duration = 0.5
        self.phrase_threshold = 0.3

    def adjust_for_ambient_noise(self, source, duration=1):
        print(f"   🎛️  [MIC] ambient-noise calibration skipped ({duration}s)")

    def listen(self, source, timeout=None, phrase_time_limit=None, **kwargs):
        text = PROMPTER.get()
        _state["last_text"] = text
        return _MockAudioData(text)


class _MockMicrophone:
    def __init__(self, device_index=None, **kwargs):
        self.device_index = device_index

    def __enter__(self):
        print("🎙️  [MOCK] microphone opened (keyboard input)")
        return self

    def __exit__(self, *exc):
        return False

    @staticmethod
    def list_microphone_names():
        return ["Mock Microphone (keyboard)"]


def _build_speech_recognition():
    sr = types.ModuleType("speech_recognition")
    sr.Recognizer = _MockRecognizer
    sr.Microphone = _MockMicrophone
    sr.AudioData = _MockAudioData

    class WaitTimeoutError(Exception):
        pass

    class UnknownValueError(Exception):
        pass

    class RequestError(Exception):
        pass

    sr.WaitTimeoutError = WaitTimeoutError
    sr.UnknownValueError = UnknownValueError
    sr.RequestError = RequestError
    sr.__mock__ = True
    return sr


# --------------------------------------------------------------------------
# Fake whisper
# --------------------------------------------------------------------------

class _MockWhisperModel:
    def transcribe(self, audio=None, **kwargs):
        return {"text": _state["last_text"], "language": "en"}


def _build_whisper():
    whisper = types.ModuleType("whisper")

    def load_model(name="base", *args, **kwargs):
        print(f"🧠 [MOCK] whisper.load_model('{name}') — nothing downloaded")
        return _MockWhisperModel()

    whisper.load_model = load_model
    whisper.__mock__ = True
    return whisper


# --------------------------------------------------------------------------
# Minimal stubs used only when the real package is absent
# --------------------------------------------------------------------------

def _build_numpy():
    np = types.ModuleType("numpy")

    class _Array(list):
        def astype(self, _dtype):
            return self

        def __truediv__(self, _other):
            return self

    np.int16 = "int16"
    np.float32 = "float32"
    np.frombuffer = lambda buf, dtype=None: _Array()
    np.__mock__ = True
    return np


def _build_fastapi():
    """Enough of FastAPI for voice_control.py to import where it isn't installed."""

    class FastAPI:
        def __init__(self, *args, **kwargs):
            self.routes = []

        def _route(self, path):
            def decorator(func):
                self.routes.append(path)
                return func
            return decorator

        def get(self, path, **kwargs):
            return self._route(path)

        def post(self, path, **kwargs):
            return self._route(path)

        def add_middleware(self, *args, **kwargs):
            pass

    fastapi = types.ModuleType("fastapi")
    fastapi.FastAPI = FastAPI

    middleware = types.ModuleType("fastapi.middleware")
    cors = types.ModuleType("fastapi.middleware.cors")

    class CORSMiddleware:
        pass

    cors.CORSMiddleware = CORSMiddleware
    middleware.cors = cors
    fastapi.middleware = middleware
    fastapi.__mock__ = True
    return fastapi, middleware, cors


def _build_uvicorn():
    uvicorn = types.ModuleType("uvicorn")

    def run(app, host="0.0.0.0", port=8000, **kwargs):
        print(f"🌐 [MOCK] uvicorn.run() skipped ({host}:{port}) — install fastapi to serve for real")

    uvicorn.run = run
    uvicorn.__mock__ = True
    return uvicorn


def _build_ollama():
    ollama = types.ModuleType("ollama")

    def chat(*args, **kwargs):
        raise RuntimeError("ollama is not installed (mock stub)")

    ollama.chat = chat
    ollama.__mock__ = True
    return ollama


def _have(module_name):
    try:
        return importlib.util.find_spec(module_name) is not None
    except (ImportError, ValueError):
        return False


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def install():
    """Swap the fakes into sys.modules. Call before importing project code."""
    global OLLAMA_AVAILABLE

    pygame, mixer = _build_pygame()
    sys.modules["pygame"] = pygame
    sys.modules["pygame.mixer"] = mixer
    sys.modules["speech_recognition"] = _build_speech_recognition()
    sys.modules["whisper"] = _build_whisper()

    if not _have("numpy"):
        sys.modules["numpy"] = _build_numpy()

    if not _have("fastapi"):
        fastapi, middleware, cors = _build_fastapi()
        sys.modules["fastapi"] = fastapi
        sys.modules["fastapi.middleware"] = middleware
        sys.modules["fastapi.middleware.cors"] = cors
    if not _have("uvicorn"):
        sys.modules["uvicorn"] = _build_uvicorn()

    OLLAMA_AVAILABLE = _have("ollama")
    if not OLLAMA_AVAILABLE:
        # intent_parser.py imports ollama at module level; keep that import working.
        sys.modules["ollama"] = _build_ollama()

    print("🧪 Mock hardware installed: no microphone, no speakers, no Whisper.")

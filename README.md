# Tangent Lab - Robotic Arm Voice Recognition

Lawrence Kim, Sarah Pratt, Yecheng Wang, Flora Jin

[Instruction docs](https://docs.google.com/document/d/1kTuhBVUUAJrRFOe4E4hBuK8t0crEJOflfly36srfFaI/edit?tab=t.0)

[Sajan Work Progress Track Doc](https://docs.google.com/document/d/14VDSbiGP7JsEkdMbIqouRZWuRzqMW7q0UEdbqUpfDkA/edit?tab=t.0)

---

### Using `voice-llm.py` and `voice-text.py`

To test the LLM parsing with speaker:
!! Before starting ensure the index is updated on voice_text.py with the correct speaker index !!
- Terminal 1: run `roscore` - allows files to talk to each other
- Terminal 2: Run the ollama server `ollama serve`
- Terminal 3: Run the python script `python3 voice_text.py`
- Terminal 4: Run the python script `python3 voice_llm.py`
- Terminal 5: Run `rostopic echo /speaker/command` to view what the command the LLM outputted

After running all the terminals in order, speak into the microphone.
Terminal 5 should output either `Sending Command: PLAY_SOUND` or `Intent Unclear`

---

### TODO list
- [ ] Need to connect speaker via bluetooth or AUX cord to Alienware laptop

---

### Flora Feb 28th Update
- Changed voice_llm.py to detect for speaker control phrases
- Now if user asks to play sound or stop sound, LLM outputs correct command

### Flora Feb 5th Update
- Added file `voice-llm.py` to test LLM connection for parsing text
- Updated `voice-text.py` to connect via ROS Node to publish text to LLM
- Was able to successfully test that LLM can parse "move forward a little" and "go home" as "HOME" and "FORWARD"

### Flora Feb 1st Update

- Added file `speech-to-text.py` to use Whisper for using microphone (speech) to text
- Added file `list_mics.py` to list available microphones that pyaudio can detect

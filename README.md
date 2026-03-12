# Tangent Lab - Robotic Arm Voice Recognition

Lawrence Kim, Sarah Pratt, Yecheng Wang, Flora Jin

[Instruction docs](https://docs.google.com/document/d/1kTuhBVUUAJrRFOe4E4hBuK8t0crEJOflfly36srfFaI/edit?tab=t.0)

[Sajan Work Progress Track Doc](https://docs.google.com/document/d/14VDSbiGP7JsEkdMbIqouRZWuRzqMW7q0UEdbqUpfDkA/edit?tab=t.0)

---

### Using `voice-control.py`

To test the LLM parsing with speaker:
!! Before starting ensure the index is updated on voice_text.py with the correct speaker index !!
!! Ensure speaker is connected via Bluetooth !!
- Terminal 1: Run the ollama server `ollama serve`
- Terminal 2: Run the python script `python voice_control.py`

After running the terminals in order, speak into the microphone.

---

### TODO list
- [ ] Need to connect speaker to Alienware laptop (if possible)
- [ ] Finish ventriloquism section
- [ ] Update GUI

---
### Flora March 11th Update
- Refactored voice_control.py into different files for the LLM, and actions
- Able to successfully run Soundscape and Telepathy sections by playing the correct music after parsing the intent
- Added a GUI as an instruction guide for users

### Flora March 4th Update
- Added voice_control.py to combine voice_llm.py and voice_text.py into one file
- Added threading for continuous command listening
- Speaker plays music on command from microphone and stops music on command
- Currently only tested that system working on MacOS
- removed ROS nodes for simplicity

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

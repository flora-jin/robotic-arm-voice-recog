# Tangent Lab - Robotic Arm Voice Recognition

Lawrence Kim, Sarah Pratt, Yecheng Wang, Flora Jin

[Instruction docs](https://docs.google.com/document/d/1kTuhBVUUAJrRFOe4E4hBuK8t0crEJOflfly36srfFaI/edit?tab=t.0)

[Sajan Work Progress Track Doc](https://docs.google.com/document/d/14VDSbiGP7JsEkdMbIqouRZWuRzqMW7q0UEdbqUpfDkA/edit?tab=t.0)

---

### Using `voice-llm.py` and `voice-text.py`
!! Have not tested with microphone yet as alienware laptop does not have microphone access !!

To test the LLM parsing:
- Terminal 1: run `roscore` - allows files to talk to each other
- Terminal 2: Run the ollama server `ollama serve`
- Terminal 3: Run the python script `python3 voice-llm.py`
- Terminal 4: Run `rostopic echo /robot/command` to view what the robot would receive via ROS Node
- Terminal 5: Will simulate the microphone input can run for example `rostopic pub -1 /voice/raw_text std_msgs/String "{data: 'move forward a little'}"`

After running all the terminals in order, Terminal 4 should output something like `data: "FORWARD"` or `data: "UNKNOWN"` for unknown commands

---

### TODO list
- [ ] Need to test on alienware laptop to see if speech to text is working correctly on ubuntu 22.04

---

### Flora Feb 5th Update
- Added file `voice-llm.py` to test LLM connection for parsing text
- Updated `voice-text.py` to connect via ROS Node to publish text to LLM
- Was able to successfully test that LLM can parse "move forward a little" and "go home" as "HOME" and "FORWARD"

### Flora Feb 1st Update

- Added file `speech-to-text.py` to use Whisper for using microphone (speech) to text
- Added file `list_mics.py` to list available microphones that pyaudio can detect

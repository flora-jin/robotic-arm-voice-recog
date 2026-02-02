# Tangent Lab - Robotic Arm Voice Recognition

Lawrence Kim, Sarah Pratt, Yecheng Wang, Flora Jin

[Instruction docs](https://docs.google.com/document/d/1kTuhBVUUAJrRFOe4E4hBuK8t0crEJOflfly36srfFaI/edit?tab=t.0)

[Sajan Work Progress Track Doc](https://docs.google.com/document/d/14VDSbiGP7JsEkdMbIqouRZWuRzqMW7q0UEdbqUpfDkA/edit?tab=t.0)

---

### Whisper - speech to text
To use the speech-to-text.py, follow the start-up instructions on the Whisper official github repository
https://github.com/openai/whisper

### TODO list
- [ ] Need to test on alienware laptop to see if speech to text is working correctly on ubuntu 22.04
- [ ] Write LLM connection in order to parse text from Whisper
- [ ] ROSNode connection using Topics

---

### Flora Feb 1st Update

- Added file `speech-to-text.py` to use Whisper for using microphone (speech) to text
- Added file `list_mics.py` to list available microphones that pyaudio can detect

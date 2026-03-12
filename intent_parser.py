import ollama

# Valid Commands
VALID_COMMANDS = ["PLAY_MOZART", "PLAY_NEWS", "PLAY_WHITE_NOISE", "STOP_SOUND",
                  "SET_TIMER", "SEND_REMINDER", "WISH_BIRTHDAY"]

def ask_ollama(user_text):
    """Classify intent using Ollama"""
    try:
        response = ollama.chat(
            model='llama3',
            messages=[
                {
                    'role': 'system',
                    'content': """
You are an intent classifier for a voice-controlled speaker.

Valid outputs:
PLAY_MOZART
PLAY_NEWS
PLAY_WHITE_NOISE
STOP_SOUND
SET_TIMER
SEND_REMINDER
WISH_BIRTHDAY
UNKNOWN

Rules:
- Output ONLY one of the exact valid outputs above.
- No punctuation.
- If the user asks to stop, lower the volume, says "I don't want to hear", "shut up", or anything negative about the sound: output STOP_SOUND.
- If the user asks for Mozart, classical, or music: output PLAY_MOZART.
- If the user asks for news or talking: output PLAY_NEWS.
- If the user asks for white noise or static: output PLAY_WHITE_NOISE.
- If the user asks to set a timer to notify them: output SET_TIMER.
- If the user asks to send a meeting reminder: output SEND_REMINDER.
- If the user asks to wish someone a happy birthday: output WISH_BIRTHDAY.
- If the intent is unclear, output UNKNOWN.
"""
                },
                {'role': 'user', 'content': user_text}
            ],
            options={"temperature": 0}
        )

        raw = response['message']['content']
        command = raw.strip().split()[0].replace(".", "").upper()

        return command if command in VALID_COMMANDS else "UNKNOWN"

    except Exception as e:
        print(f"LLM Error: {e}")
        return "UNKNOWN"

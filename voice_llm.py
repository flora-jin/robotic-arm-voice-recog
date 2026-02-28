#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import ollama
import json

# 1. Define the Robot's "Menu" (Valid Commands)
VALID_COMMANDS = ["PLAY_SOUND", "STOP_SOUND"]

def ask_ollama(user_text):
    try:
        response = ollama.chat(
            model='llama3',
            messages=[
                {
                    'role': 'system',
                    'content': """
You are an intent classifier.

Valid outputs:
PLAY_SOUND
STOP_SOUND
UNKNOWN

Rules:
- Output ONLY one word from above.
- No explanation.
- No punctuation.
"""
                },
                {
                    'role': 'user',
                    'content': user_text
                }
            ],
            options={"temperature": 0}
        )

        raw_output = response['message']['content']
        command = raw_output.strip().split()[0].replace(".", "").upper()

        if command in VALID_COMMANDS:
            return command
        else:
            return "UNKNOWN"

    except Exception as e:
        print(f"LLM Error: {e}")
        return "UNKNOWN"

def callback(msg):
    raw_text = msg.data
    print(f"🧠 Processing: '{raw_text}'")
    
    # 1. Get Command from Ollama
    command_id = ask_ollama(raw_text)
    
    # 2. Publish Strict Command
    if command_id != "UNKNOWN":
        print(f"🚀 Sending Command: {command_id}")
        cmd_pub.publish(command_id)
    else:
        print("🤷 Intent Unclear")

if __name__ == '__main__':
    rospy.init_node('llm_brain')
    
    # Subscribe to Ears
    rospy.Subscriber('/voice/raw_text', String, callback)
    
    # Publish to Robot
    cmd_pub = rospy.Publisher('/speaker/command', String, queue_size=10)
    
    print("🧠 Brain Node Ready. Waiting for text...")
    rospy.spin()
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import ollama
import json

# 1. Define the Robot's "Menu" (Valid Commands)
VALID_COMMANDS = ["HOME", "FORWARD", "UP", "STOP"]

def ask_ollama(user_text):
    # The prompt forces Ollama to act as a classifier, not a chatbot
    system_prompt = f"""
    You are a robot controller. Map the user's speech to one of these EXACT commands: {VALID_COMMANDS}.
    
    Rules:
    - Output ONLY the command word.
    - If the user asks for something impossible, output "UNKNOWN".
    - Do not write sentences.
    
    User: "{user_text}"
    Command:
    """
    
    try:
        response = ollama.chat(model='llama3', messages=[
            {'role': 'user', 'content': system_prompt}
        ])
        # Clean up output (remove whitespace/newlines)
        command = response['message']['content'].strip().upper()
        
        # Double check it's in our list (Safety check #1)
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
    cmd_pub = rospy.Publisher('/robot/command', String, queue_size=10)
    
    print("🧠 Brain Node Ready. Waiting for text...")
    rospy.spin()
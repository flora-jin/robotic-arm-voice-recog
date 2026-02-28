from voice_llm import ask_ollama

while True:
    text = input("Type something: ")
    print("Intent:", ask_ollama(text))
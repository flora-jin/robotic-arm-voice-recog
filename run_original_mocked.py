"""Run the UNMODIFIED voice_control.py main loop under mocked hardware."""
import mock_io
mock_io.install()
import voice_control

if __name__ == "__main__":
    try:
        voice_control.main()
    except (KeyboardInterrupt, mock_io.DemoExit):
        print("\n👋 Demo ended.")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from gtts import gTTS
import os
import tempfile

# Braille to alphabet mapping (0-5 system)
braille_to_alphabet = {
    (0,): 'A',
    (0, 3): 'B',
    (0, 1): 'C',
    (0, 1, 4): 'D',
    (0, 4): 'E',
    (0, 1, 3): 'F',
    (0, 1, 3, 4): 'G',
    (0, 3, 4): 'H',
    (1, 3): 'I',
    (1, 3, 4): 'J',
    (0, 2): 'K',
    (0, 2, 3): 'L',
    (0, 1, 2): 'M',
    (0, 1, 2, 4): 'N',
    (0, 2, 4): 'O',
    (0, 1, 2, 3): 'P',
    (0, 1, 2, 3, 4): 'Q',
    (0, 2, 3, 4): 'R',
    (1, 2, 3): 'S',
    (1, 2, 3, 4): 'T',
    (0, 2, 5): 'U',
    (0, 2, 3, 5): 'V',
    (1, 3, 4, 5): 'W',
    (0, 1, 2, 5): 'X',
    (0, 1, 2, 4, 5): 'Y',
    (0, 2, 4, 5): 'Z',
    (6,): ' ',  # Space character
}

class AuditoryFeedbackNode(Node):
    def __init__(self):
        super().__init__('auditory_feedback_node')

        # Subscribe to the /button_states topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/button_states',
            self.listener_callback,
            10)
        
        # Track the previous valid letter
        self.previous_letter = None

    def listener_callback(self, msg):
        """
        Callback for processing received button states and providing auditory feedback.
        """
        data = msg.data  # List of integers (button states)
        
        if not data:
            return  # Ignore empty data
        
        for button_state in data:
            # Convert button_state to tuple if it's a single integer
            button_state = (button_state,) if isinstance(button_state, int) else tuple(sorted(button_state))
            
            # Get the mapped alphabet
            alphabet = braille_to_alphabet.get(button_state, None)
            
            if alphabet:
                # Speak the valid letter
                self.speak(alphabet)
                self.previous_letter = alphabet
            elif self.previous_letter:
                # Repeat the previous letter if no valid mapping
                self.speak(self.previous_letter)

    def speak(self, text):
        """
        Use gTTS to convert the given text to speech and play it, if valid.
        """
        if not text or not isinstance(text, str):  # Ensure text is valid
            self.get_logger().warn("Invalid text received for speech synthesis. Staying silent.")
            return  # Skip processing

        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True) as temp_audio:
                temp_audio_path = temp_audio.name + ".mp3"
                tts.save(temp_audio_path)
                os.system(f"mpg321 {temp_audio_path} > /dev/null 2>&1")  # Suppress logs
        except Exception as e:
            self.get_logger().warn(f"Repeating previous letter: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Initialize and spin the auditory feedback node
    auditory_feedback_node = AuditoryFeedbackNode()
    try:
        rclpy.spin(auditory_feedback_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        auditory_feedback_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import tkinter as tk
from datetime import datetime
import csv
from gtts import gTTS
import os
import tempfile


# Braille to alphabet mapping (0-5 system)
braille_to_alphabet = {
    (0,): 'A',
    (0, 1): 'B',
    (0, 3): 'C',
    (0, 3, 4): 'D',
    (0, 4): 'E',
    (0, 1, 3): 'F',
    (0, 1, 3, 4): 'G',
    (0, 1, 4): 'H',
    (1, 3): 'I',
    (1, 3, 4): 'J',
    (0, 2): 'K',
    (0, 1, 2): 'L',
    (0, 2, 3): 'M',
    (0, 2, 3, 4): 'N',
    (0, 2, 4): 'O',
    (0, 1, 2, 3): 'P',
    (0, 1, 2, 3, 4): 'Q',
    (0, 1, 2, 4): 'R',
    (1, 2, 3): 'S',
    (1, 2, 3, 4): 'T',
    (0, 2, 5): 'U',
    (0, 1, 2, 5): 'V',
    (1, 3, 4, 5): 'W',
    (0, 2, 3, 5): 'X',
    (0, 2, 3, 4, 5): 'Y',
    (0, 2, 4, 5): 'Z',
    (6,): ' ',
}

# Initialize CSV file for logging
csv_file = "typed_alphabets_ros2.csv"
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Typed Alphabet"])

# GUI for displaying typed alphabets
class TypingGUI:
    def __init__(self, root):
        self.root = root
        self.text_area = tk.Text(root, height=10, width=50, font=("Helvetica", 16))
        self.text_area.pack()
        self.root.title("Braille to Alphabet Typing")

    def update_text(self, char):
        self.text_area.insert(tk.END, char)

# ROS 2 Node
class BrailleToAlphabetNode(Node):
    def __init__(self, gui):
        super().__init__('braille_to_alphabet_node')
        self.gui = gui
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/button_states',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
    
       

    def listener_callback(self, msg):
        pressed_buttons = tuple(sorted(msg.data))
        if pressed_buttons in braille_to_alphabet:
            alphabet = braille_to_alphabet[pressed_buttons]
            self.get_logger().info(f"Pressed buttons {pressed_buttons} -> {alphabet}")
            self.speak(alphabet)
            self.gui.update_text(alphabet)  # Update GUI
            self.write_to_csv(alphabet)    # Log to CSV
        else:
            self.get_logger().warn(f"Pressed buttons {pressed_buttons} have no mapping!")

    def write_to_csv(self, alphabet):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, alphabet])

    def speak(self, text):
            """
            Use gTTS to convert the given text to speech and play it, if valid.
            """
            if not text or not isinstance(text, str):  # Ensure text is valid
                self.get_logger().warn("Invalid text received for speech synthesis. Staying silent.")
                return  # Skip processing

            try:
                tts = gTTS(text=text, lang='en-in')
                with tempfile.NamedTemporaryFile(delete=True) as temp_audio:
                    temp_audio_path = temp_audio.name + ".mp3"
                    tts.save(temp_audio_path)
                    os.system(f"mpg321 {temp_audio_path} > /dev/null 2>&1")  # Suppress logs
            except Exception as e:
                self.get_logger().warn(f"Repeating previous letter: {e}")

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = TypingGUI(root)
    braille_node = BrailleToAlphabetNode(gui)

    # Run the ROS 2 node in a separate thread
    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(braille_node,))
    ros_thread.start()

    # Start the GUI
    root.mainloop()

    # Cleanup after GUI closes
    braille_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()


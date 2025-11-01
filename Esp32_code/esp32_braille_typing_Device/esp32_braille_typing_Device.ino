#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <vector>
#include <std_msgs/msg/int32_multi_array.h>

// Wi-Fi credentials (replace with your network's credentials)
const char *ssid = "Network";
const char *password = "12345678";

// Button pin definitions
const int buttonPins[] = { 2, 3, 4, 5, 6, 7, 8, 9, 10 };  // Define your GPIO pins
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

std::vector<int> buttonList;  // List to store pressed button IDs
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg;

// Micro-ROS allocator and node
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// Timing
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 500;  // Publish interval in ms

void setup() {
  Serial.begin(115200);

  // Button setup
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Micro-ROS setup
  set_microros_wifi_transports("Network", "12345678", "10.159.93.248", 8888);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_button_node", "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "button_states");

  // Initialize message
  msg.data.capacity = 7;  // Maximum number of buttons
  msg.data.size = 0;
  msg.data.data = (int32_t *)malloc(msg.data.capacity * sizeof(int32_t));
}

void loop() {
  // Check button states
  bool anyButtonPressed = false;
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {  // Button is pressed
      if (std::find(buttonList.begin(), buttonList.end(), i) == buttonList.end()) {
        buttonList.push_back(i);  // Add button index to the list
        Serial.print("Button ");
        Serial.print(i);
        Serial.println(" pressed.");
      }
      anyButtonPressed = true;
    }
  }

  // Publish and reset the list if no buttons are pressed
  if (!anyButtonPressed && !buttonList.empty()) {
    Serial.println("No button pressed. Publishing and resetting the list.");

    // Populate message with current button list
    msg.data.size = buttonList.size();
    for (size_t i = 0; i < buttonList.size(); i++) {
      msg.data.data[i] = buttonList[i];
    }

    // Publish the message
    rcl_publish(&publisher, &msg, NULL);

    // Clear the button list
    buttonList.clear();
  }
}

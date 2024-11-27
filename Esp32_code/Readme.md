When any one of the button is pressed:
   1. Get the button index when pressed. (0 to 6) 
   2.  Add the index to a list.



When NO button is pressed:
   1. Publish the list to a topic (/button_states) using micro-ROS.
   2. Reset the list. 

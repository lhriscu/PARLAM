## Operator: Listening and Speaking via ROS 2 Services

The following instructions are a simple guide to use the ROS 2 nodes of PARLAM for human control. Please note that this is not a complete guide since further integration with platforms such as Rviz are required to automatize the process.

1. Listening to User Input:
ros2 service call /speech_input parlam_interfaces/srv/Input

This will return the user's input transcribed from audio to text.

2. Send Audio Response:

ros2 service call /speech_output parlam_interfaces/srv/Output "{answer: 'INSERT TEXT'}"

This will send the text to the TTS audio system, which will convert it to speech and play it back.
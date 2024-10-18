# hello_stretch-24
 
## alexa.py – Alexa Node
This serves as both a web server for a teleoperation interface and the back end for an Alexa Skill and a ROS2 node 
As the Flask server and node must run in separate threads, multi-threading is used

## stretch_node.py – Stretch Node
Subscribes to the intent topics
Uses stretch body to send commands to the robot
Could publish information from robot sensors so the Alexa Node can inform its response to the user

## util.py
Helper for both Nodes

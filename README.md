# Amazon Alexa Interface for Hello Stretch Robot, done while working in the UMass Lowell ARA Lab.
 
## Summary
Two ROS2 Nodes used to create an Amazon Alexa and web interface for a Hello Stretch Robot.

## Amazon Alexa Node and Web Interface - alexa.py
Acts as a backend for an Amazon Alexa Skill and hosts a Flask app web interface locally. Processes intent requests and publishes information about the user's choices from Alexa and the web interface.

Ngrok can be used to tunnel to a https address, which can be used as the Endpoint in the ALexa Skill.

## Hello Stretch Node - stretch_node.py
Controls the Stretch robot using stretch_body from Hello Robot in different command groupings depending on the requested intent. Subscribes the topics that inform about the user requests sent from the the Amazon Alexa node.

## util.py
Helper file for both nodes.

#!/bin/bash

echo "Running the MiRo Gamepad Controller... press ctrl+c to exit."

# this line is a lazy fix used to prevent a bug in pygame from continuously printing debugging messages
/home/matt/miro_gamepad_controller/controller.py > /dev/null

# running the controller program
# python controller.py

function ctrl_c() {
	echo "Hope you had fun!"
}

# trap ctrl-c and call ctrl_c()
trap ctrl_c EXIT

#!/bin/bash

echo "Running the MiRo Gamepad Controller... press ctrl+c to exit."

# this line is a lazy fix used to prevent a bug in pygame from continuously printing debugging messages
$(pwd)/controller.py > /dev/null

# running the controller program
# python controller.py

function exiting() {
	printf "\nHope you had fun!\n"
}

# trap ctrl-c and call ctrl_c()
trap exiting EXIT

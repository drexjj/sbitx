#!/bin/bash

# Check if the keypad is already running
if [ -f "/tmp/frequency_keypad.lock" ]; then
    # Keypad is already running, try to bring it to focus
    # This uses xdotool which is commonly available on Raspberry Pi
    if command -v xdotool &> /dev/null; then
        # Try to find and focus the window
        xdotool search --name "Frequency Keypad" windowactivate
    else
        # If xdotool is not available, we can't focus the window
        # but we don't want to launch another instance
        echo "Keypad is already running"
    fi
else
    # Keypad is not running, launch it
    python3 /home/pi/sbitx/src/freq-direct.py &
fi

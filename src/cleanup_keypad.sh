#!/bin/bash

# Check if the keypad is running
if [ -f "/tmp/frequency_keypad.lock" ]; then
    # Remove the lock file to allow the keypad to be closed
    rm -f /tmp/frequency_keypad.lock
    
    # Find and kill any running instances of freq-direct.py
    pkill -f "python3 /home/pi/sbitx/src/freq-direct.py"
fi

#!/bin/bash
# Script to initialize xfwm4 window manager for application-specific VNC sessions
# This adds window decorations (titlebars) to applications so they can be moved/resized

# Usage: init_window_manager.sh [DISPLAY_NUM]
# Example: init_window_manager.sh 1  # For display :1

# Get the display number from command line or use default 1
DISPLAY_NUM=${1:-1}
DISPLAY_NAME=":$DISPLAY_NUM"

echo "Initializing window manager (xfwm4) for display $DISPLAY_NAME"

# Check if xfwm4 is already running on this display
if pgrep -f "xfwm4 --display $DISPLAY_NAME" > /dev/null; then
    echo "xfwm4 is already running on display $DISPLAY_NAME"
    exit 0
fi

# Start xfwm4 on the specified display
DISPLAY=$DISPLAY_NAME xfwm4 &
XFWM_PID=$!

# Wait a moment for window manager to initialize
sleep 1

# Save PID for later cleanup
echo "$XFWM_PID" > /tmp/xfwm4_${DISPLAY_NUM}.pid
echo "xfwm4 started on display $DISPLAY_NAME with PID: $XFWM_PID"

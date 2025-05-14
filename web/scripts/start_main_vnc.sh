#!/bin/bash
# Define the VNC and WebSocket ports for this application
# The webserver will read these values to properly configure the web interface
VNC_PORT=5900
WS_PORT=6080
DISPLAY_NUM=0

# Define the widget label for the web interface
WIDGET_LABEL="Main Desktop VNC"

# Define the application name and command
APP_NAME="Main Desktop"
APP_COMMAND="desktop"

# Script to start the main VNC desktop

# Check if x11vnc is already running on our port
if netstat -tuln | grep -q :$VNC_PORT; then
    echo "Port $VNC_PORT is already in use, x11vnc may already be running"
else
    # Start x11vnc on the main display, port $VNC_PORT
    x11vnc -display :$DISPLAY_NUM -rfbport $VNC_PORT -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_main.log &
    X11VNC_PID=$!
    echo "Main x11vnc started with PID: $X11VNC_PID"
    echo "$X11VNC_PID" > /tmp/main_x11vnc.pid
    
    # Make sure xfwm4 is running on the main display
    if ! pgrep -f "xfwm4 --display :$DISPLAY_NUM" > /dev/null; then
        echo "Starting xfwm4 on main display :$DISPLAY_NUM"
        DISPLAY=:$DISPLAY_NUM xfwm4 --daemon &
        XFWM_PID=$!
        echo "xfwm4 started with PID: $XFWM_PID"
        echo "$XFWM_PID" > /tmp/xfwm4_0.pid
        # Wait a moment for window manager to initialize
        sleep 1
    else
        echo "xfwm4 already running on display :$DISPLAY_NUM"
    fi
fi

# Start NoVNC proxy for the main VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "Main VNC desktop started"

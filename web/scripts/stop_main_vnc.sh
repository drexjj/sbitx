#!/bin/bash
# Define the application name
APP_NAME="Main Desktop"

# Define the VNC and WebSocket ports for this application
VNC_PORT=5900
WS_PORT=6080

# Script to stop the main VNC desktop and NoVNC proxy

# Stop x11vnc
pid=$(cat /tmp/main_x11vnc.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/main_x11vnc.pid
    echo "$APP_NAME x11vnc stopped"
else
    # Try to find and kill any running x11vnc processes on our port
    pid=$(ps aux | grep "x11vnc.*-rfbport $VNC_PORT" | grep -v grep | awk '{print $2}')
    if [ -n "$pid" ]; then
        kill $pid 2>/dev/null
        echo "Found and stopped x11vnc on port $VNC_PORT"
    fi
fi

# Stop the NoVNC proxy for the main VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"

#!/bin/bash
# Script to start the main VNC desktop on port 5900 and NoVNC proxy on port 6080

# Check if x11vnc is already running on port 5900
if netstat -tuln | grep -q :5900; then
    echo "Port 5900 is already in use, x11vnc may already be running"
else
    # Start x11vnc on the main display :0, port 5900
    x11vnc -display :0 -rfbport 5900 -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_main.log &
    X11VNC_PID=$!
    echo "Main x11vnc started with PID: $X11VNC_PID"
    echo "$X11VNC_PID" > /tmp/main_x11vnc.pid
fi

# Start NoVNC proxy for the main VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh 5900

echo "Main VNC desktop started"

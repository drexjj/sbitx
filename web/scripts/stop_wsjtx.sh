#!/bin/bash
# Define the application name and command
APP_NAME="WSJT-X"
APP_COMMAND="wsjtx"

# Define the VNC and WebSocket ports for this application
VNC_PORT=5901
WS_PORT=6081

# Stop $APP_NAME
pid=$(cat /tmp/wsjtx_app.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/wsjtx_app.pid
fi

# Stop x11vnc
pid=$(cat /tmp/wsjtx_x11vnc.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/wsjtx_x11vnc.pid
fi

# Stop Xvfb
pid=$(cat /tmp/wsjtx_xvfb.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/wsjtx_xvfb.pid
fi

# Stop the NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"
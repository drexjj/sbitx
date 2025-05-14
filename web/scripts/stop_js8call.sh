#!/bin/bash
# Define the application name
APP_NAME="JS8Call"

# Define the VNC and WebSocket ports for this application
VNC_PORT=5903
WS_PORT=6083

# Stop $APP_NAME
pid=$(cat /tmp/js8call_app.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/js8call_app.pid
fi

# Stop x11vnc
pid=$(cat /tmp/js8call_x11vnc.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/js8call_x11vnc.pid
fi

# Stop Xvfb
pid=$(cat /tmp/js8call_xvfb.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/js8call_xvfb.pid
fi

# Stop the NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"

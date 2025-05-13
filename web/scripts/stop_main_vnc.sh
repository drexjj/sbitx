#!/bin/bash
# Script to stop the main VNC desktop and NoVNC proxy

# Stop x11vnc
pid=$(cat /tmp/main_x11vnc.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/main_x11vnc.pid
    echo "Main x11vnc stopped"
else
    # Try to find and kill any running x11vnc processes on port 5900
    pid=$(ps aux | grep "x11vnc.*-rfbport 5900" | grep -v grep | awk '{print $2}')
    if [ -n "$pid" ]; then
        kill $pid 2>/dev/null
        echo "Found and stopped x11vnc on port 5900"
    fi
fi

# Stop the NoVNC proxy for the main VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh 5900

echo "Main VNC desktop stopped"

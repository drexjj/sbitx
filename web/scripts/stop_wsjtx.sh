#!/bin/bash
# Stop WSJT-X
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
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh 5901

echo "WSJT-X stopped"
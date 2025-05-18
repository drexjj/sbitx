#!/bin/bash
# Define the application name
APP_NAME="wsjtx_ft8"

# Define the VNC and WebSocket ports for this application
VNC_PORT=5901
WS_PORT=6081

# Stop $APP_NAME
pid=$(cat /tmp/${APP_NAME}_app.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/${APP_NAME}_app.pid
fi

# Stop jt9
jt9_pid=$(pgrep jt9)
if [ -n "$jt9_pid" ]; then
    kill $jt9_pid 2>/dev/null
    echo "jt9 stopped"
else
    echo "jt9 was not running"
fi

# Stop x11vnc
pid=$(cat /tmp/${APP_NAME}_x11vnc.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/${APP_NAME}_x11vnc.pid
fi

# Stop Xvfb
pid=$(cat /tmp/${APP_NAME}_xvfb.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/${APP_NAME}_xvfb.pid
fi

# Stop wmctrl
pid=$(cat /tmp/${APP_NAME}_wmctrl.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/${APP_NAME}_wmctrl.pid
fi


# Stop the NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"
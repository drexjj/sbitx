#!/bin/bash
# Define the application name
APP_NAME="cqrlog"

# Define the VNC and WebSocket ports for this application
VNC_PORT=5902
WS_PORT=6082
DISPLAY_NUM=2

# Stop $APP_NAME
pid=$(cat /tmp/${APP_NAME}_app.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/${APP_NAME}_app.pid
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

# Stop xfwm4
pid=$(cat /tmp/xfwm4_${DISPLAY_NUM}.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/xfwm4_${DISPLAY_NUM}.pid
fi

# Stop the NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"
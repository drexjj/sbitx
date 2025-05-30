#!/bin/bash
# Define the application name and command ** APP_NAME MUST match the script name (after the start_  prefix) for proper app-status API tracking **
APP_NAME="gridtracker"


# Define the VNC and WebSocket ports for this application
VNC_PORT=5910
WS_PORT=6090
DISPLAY_NUM=10
# Script to stop the application and clean up resources

# Check if the application is running
if [ -f /tmp/${APP_NAME}_app.pid ]; then
    APP_PID=$(cat /tmp/${APP_NAME}_app.pid)
    if ps -p $APP_PID > /dev/null; then
        echo "Stopping $APP_NAME (PID: $APP_PID)"
        kill $APP_PID
    else
        echo "$APP_NAME process not found, but PID file exists"
    fi
    rm -f /tmp/${APP_NAME}_app.pid
fi

# Stop x11vnc
if [ -f /tmp/${APP_NAME}_x11vnc.pid ]; then
    X11VNC_PID=$(cat /tmp/${APP_NAME}_x11vnc.pid)
    if ps -p $X11VNC_PID > /dev/null; then
        echo "Stopping x11vnc (PID: $X11VNC_PID)"
        kill $X11VNC_PID
    else
        echo "x11vnc process not found, but PID file exists"
    fi
    rm -f /tmp/${APP_NAME}_x11vnc.pid
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

# Stop NoVNC proxy
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"

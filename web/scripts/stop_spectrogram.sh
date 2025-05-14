#!/bin/bash
# Define the application name
APP_NAME="Spectrogram Generator"


# Define the VNC and WebSocket ports for this application
VNC_PORT=5910
WS_PORT=6090

# Script to stop the application and clean up resources

# Check if the application is running
if [ -f /tmp/spectrogram_app.pid ]; then
    APP_PID=$(cat /tmp/spectrogram_app.pid)
    if ps -p $APP_PID > /dev/null; then
        echo "Stopping $APP_NAME (PID: $APP_PID)"
        kill $APP_PID
    else
        echo "$APP_NAME process not found, but PID file exists"
    fi
    rm -f /tmp/spectrogram_app.pid
fi

# Stop x11vnc
if [ -f /tmp/spectrogram_x11vnc.pid ]; then
    X11VNC_PID=$(cat /tmp/spectrogram_x11vnc.pid)
    if ps -p $X11VNC_PID > /dev/null; then
        echo "Stopping x11vnc (PID: $X11VNC_PID)"
        kill $X11VNC_PID
    else
        echo "x11vnc process not found, but PID file exists"
    fi
    rm -f /tmp/spectrogram_x11vnc.pid
fi

# Stop Xvfb
if [ -f /tmp/spectrogram_xvfb.pid ]; then
    XVFB_PID=$(cat /tmp/spectrogram_xvfb.pid)
    if ps -p $XVFB_PID > /dev/null; then
        echo "Stopping Xvfb (PID: $XVFB_PID)"
        kill $XVFB_PID
    else
        echo "Xvfb process not found, but PID file exists"
    fi
    rm -f /tmp/spectrogram_xvfb.pid
fi

# Stop the NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/stop_novnc_proxy.sh $VNC_PORT $WS_PORT

echo "$APP_NAME stopped"

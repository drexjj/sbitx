#!/bin/bash
# Stop other apps
/home/pi/sbitx/web/scripts/stop_wsjtx.sh
/home/pi/sbitx/web/scripts/stop_js8call.sh

# Check if FLDigi is already running
pid=$(pgrep -f fldigi)
if [ -n "$pid" ]; then
    echo "FLDigi is already running"
    exit 0
fi

# Start Xvfb for display :2
Xvfb :2 -screen 0 1600x900x24 &
XVFB_PID=$!

# Wait for Xvfb to start
sleep 1

# Start x11vnc on display :2, port 5902
# Start without SSL - we'll handle encryption at the webserver level
x11vnc -display :2 -rfbport 5902 -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_fldigi.log &
X11VNC_PID=$!

# Start FLDigi on display :2
DISPLAY=:2 fldigi &
APP_PID=$!

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/fldigi_xvfb.pid
echo "$X11VNC_PID" > /tmp/fldigi_x11vnc.pid
echo "$APP_PID" > /tmp/fldigi_app.pid

echo "FLDigi started"
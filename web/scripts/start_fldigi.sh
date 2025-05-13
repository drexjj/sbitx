#!/bin/bash
# Stop other apps
/home/pi/sbitx/web/scripts/stop_wsjtx.sh
/home/pi/sbitx/web/scripts/stop_js8call.sh

# Make sure the scripts are executable
chmod +x /home/pi/sbitx/web/scripts/start_novnc_proxy.sh
chmod +x /home/pi/sbitx/web/scripts/stop_novnc_PROXY.sh

# Check if FLDigi is already running
pid=$(pgrep -x fldigi)
if [ -n "$pid" ]; then
    echo "FLDigi is already running with PID: $pid" >> /home/pi/x11vnc_fldigi.log
    ps -p $pid -o cmd= >> /home/pi/x11vnc_fldigi.log
    exit 0
fi

# Start Xvfb for display :2
Xvfb :2 -screen 0 1280x1024x16 &
XVFB_PID=$!
echo "Xvfb PID: $XVFB_PID" >> /home/pi/x11vnc_fldigi.log

# Wait for Xvfb to start
sleep 1

# Check if port 5902 is in use
if netstat -tuln | grep -q :5902; then
    echo "Port 5902 is already in use, attempting to kill process" >> /home/pi/x11vnc_fldigi.log
    fuser -k 5902/tcp
    sleep 1
fi

# Start x11vnc on display :2, port 5902
x11vnc -display :2 -rfbport 5902 -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_fldigi.log &
X11VNC_PID=$!
echo "x11vnc PID: $X11VNC_PID" >> /home/pi/x11vnc_fldigi.log

# Initialize window manager to add titlebars/decorations
/home/pi/sbitx/web/scripts/init_window_manager.sh 2

# Start FLDigi on display :2
DISPLAY=:2 fldigi &
APP_PID=$!
echo "FLDigi PID: $APP_PID" >> /home/pi/x11vnc_fldigi.log

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/fldigi_xvfb.pid
echo "$X11VNC_PID" > /tmp/fldigi_x11vnc.pid
echo "$APP_PID" > /tmp/fldigi_app.pid

echo "FLDigi started"

# Start NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh 5902
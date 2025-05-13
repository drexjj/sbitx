#!/bin/bash
# Stop other apps
/home/pi/sbitx/web/scripts/stop_wsjtx.sh
/home/pi/sbitx/web/scripts/stop_fldigi.sh

# Make sure the scripts are executable
chmod +x /home/pi/sbitx/web/scripts/start_novnc_proxy.sh
chmod +x /home/pi/sbitx/web/scripts/stop_novnc_proxy.sh

# Check if JS8Call is already running
pid=$(pgrep -x js8call)
if [ -n "$pid" ]; then
    echo "JS8Call is already running with PID: $pid" >> /home/pi/x11vnc_js8call.log
    ps -p $pid -o cmd= >> /home/pi/x11vnc_js8call.log
    exit 0
fi

# Start Xvfb for display :3
Xvfb :3 -screen 0 1280x1024x16 &
XVFB_PID=$!
echo "Xvfb PID: $XVFB_PID" >> /home/pi/x11vnc_js8call.log

# Wait for Xvfb to start
sleep 1

# Check if port 5903 is in use
if netstat -tuln | grep -q :5903; then
    echo "Port 5903 is already in use, attempting to kill process" >> /home/pi/x11vnc_js8call.log
    fuser -k 5903/tcp
    sleep 1
fi

# Start x11vnc on display :3, port 5903
x11vnc -display :3 -rfbport 5903 -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_js8call.log &
X11VNC_PID=$!
echo "x11vnc PID: $X11VNC_PID" >> /home/pi/x11vnc_js8call.log

# Initialize window manager to add titlebars/decorations
/home/pi/sbitx/web/scripts/init_window_manager.sh 3

# Start JS8Call on display :3
DISPLAY=:3 js8call &
APP_PID=$!
echo "JS8Call PID: $APP_PID" >> /home/pi/x11vnc_js8call.log

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/js8call_xvfb.pid
echo "$X11VNC_PID" > /tmp/js8call_x11vnc.pid
echo "$APP_PID" > /tmp/js8call_app.pid

echo "JS8Call started"

# Start NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh 5903
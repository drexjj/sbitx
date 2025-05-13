#!/bin/bash
# Stop other apps
/home/pi/sbitx/web/scripts/stop_fldigi.sh
/home/pi/sbitx/web/scripts/stop_js8call.sh

# Make sure the scripts are executable
chmod +x /home/pi/sbitx/web/scripts/start_novnc_proxy.sh
chmod +x /home/pi/sbitx/web/scripts/stop_novnc_proxy.sh

# Check if WSJT-X is already running
pid=$(pgrep -x wsjtx)
if [ -n "$pid" ]; then
    echo "WSJT-X is already running with PID: $pid" >> /home/pi/x11vnc_wsjtx.log
    ps -p $pid -o cmd= >> /home/pi/x11vnc_wsjtx.log
    exit 0
fi

# Start Xvfb for display :1
Xvfb :1 -screen 0 1280x1024x16 &
XVFB_PID=$!
echo "Xvfb PID: $XVFB_PID" >> /home/pi/x11vnc_wsjtx.log

# Wait for Xvfb to start
sleep 1

# Check if port 5901 is in use
if netstat -tuln | grep -q :5901; then
    echo "Port 5901 is already in use, attempting to kill process" >> /home/pi/x11vnc_wsjtx.log
    fuser -k 5901/tcp
    sleep 1
fi

# Start x11vnc on display :1, port 5901
# Start without SSL - we'll handle encryption at the webserver level
x11vnc -display :1 -rfbport 5901 -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_wsjtx.log &
X11VNC_PID=$!
echo "x11vnc PID: $X11VNC_PID" >> /home/pi/x11vnc_wsjtx.log

# Initialize window manager to add titlebars/decorations
/home/pi/sbitx/web/scripts/init_window_manager.sh 1

# Start WSJT-X on display :1
DISPLAY=:1 wsjtx &
APP_PID=$!
echo "WSJT-X PID: $APP_PID" >> /home/pi/x11vnc_wsjtx.log

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/wsjtx_xvfb.pid
echo "$X11VNC_PID" > /tmp/wsjtx_x11vnc.pid
echo "$APP_PID" > /tmp/wsjtx_app.pid

echo "WSJT-X started"

# Start NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh 5901

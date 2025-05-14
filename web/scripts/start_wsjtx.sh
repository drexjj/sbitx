#!/bin/bash
# Define the VNC and WebSocket ports for this application
# The webserver will read these values to properly configure the web interface
VNC_PORT=5901
WS_PORT=6081
DISPLAY_NUM=1

# Define the widget label for the web interface
WIDGET_LABEL="WSJT-X FT8"

# Define the application name and start command
APP_NAME="wsjtx_ft8"
APP_COMMAND="wsjtx"

# Stop other apps
/home/pi/sbitx/web/scripts/stop_fldigi.sh
/home/pi/sbitx/web/scripts/stop_js8call.sh

# Make sure the scripts are executable
chmod +x /home/pi/sbitx/web/scripts/start_novnc_proxy.sh
chmod +x /home/pi/sbitx/web/scripts/stop_novnc_proxy.sh

# Check if $APP_NAME is already running
pid=$(pgrep -x $APP_COMMAND)
if [ -n "$pid" ]; then
    echo "$APP_NAME is already running with PID: $pid" >> /home/pi/x11vnc_${APP_NAME}.log
    ps -p $pid -o cmd= >> /home/pi/x11vnc_${APP_NAME}.log
    exit 0
fi

# Start Xvfb for our display
Xvfb :$DISPLAY_NUM -screen 0 1280x1024x16 &
XVFB_PID=$!
echo "Xvfb PID: $XVFB_PID" >> /home/pi/x11vnc_${APP_NAME}.log

# Wait for Xvfb to start
sleep 1

# Check if port $VNC_PORT is in use
if netstat -tuln | grep -q :$VNC_PORT; then
    echo "Port $VNC_PORT is already in use, attempting to kill process" >> /home/pi/x11vnc_${APP_NAME}.log
    fuser -k $VNC_PORT/tcp
    sleep 1
fi

# Start x11vnc on our display, port $VNC_PORT
# Start without SSL - we'll handle encryption at the webserver level
x11vnc -display :$DISPLAY_NUM -rfbport $VNC_PORT -rfbauth /home/pi/.vnc/passwd -shared -forever -o /home/pi/x11vnc_${APP_NAME}.log &
X11VNC_PID=$!
echo "x11vnc PID: $X11VNC_PID" >> /home/pi/x11vnc_${APP_NAME}.log

# Initialize window manager to add titlebars/decorations
/home/pi/sbitx/web/scripts/init_window_manager.sh $DISPLAY_NUM

# Start $APP_NAME on our display
DISPLAY=:$DISPLAY_NUM $APP_COMMAND &
APP_PID=$!
echo "$APP_NAME PID: $APP_PID" >> /home/pi/x11vnc_${APP_NAME}.log

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/${APP_NAME}_xvfb.pid
echo "$X11VNC_PID" > /tmp/${APP_NAME}_x11vnc.pid
echo "$APP_PID" > /tmp/${APP_NAME}_app.pid

echo "$APP_NAME started"

# Start NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh $VNC_PORT $WS_PORT

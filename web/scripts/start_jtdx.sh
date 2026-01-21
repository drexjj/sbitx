#!/bin/bash
# Define the VNC and WebSocket ports for this application
# The webserver will read these values to properly configure the web interface
VNC_PORT=5902
WS_PORT=6082
DISPLAY_NUM=2

# Define the widget label for the web interface
WIDGET_LABEL="JTDX"

# Define the application name and command
APP_NAME="jtdx"
APP_COMMAND="jtdx"

# Stop other apps
/home/pi/sbitx/web/scripts/stop_wsjtx.sh
/home/pi/sbitx/web/scripts/stop_js8call.sh
/home/pi/sbitx/web/scripts/stop_mshv.sh
/home/pi/sbitx/web/scripts/stop_fldigi.sh

# Make sure the scripts are executable
chmod +x /home/pi/sbitx/web/scripts/start_novnc_proxy.sh
chmod +x /home/pi/sbitx/web/scripts/stop_novnc_proxy.sh

# Check if $APP_NAME is already running
pid=$(pgrep -x $APP_COMMAND)
if [ -n "$pid" ]; then
    echo "$APP_NAME is already running with PID: $pid" >> /tmp/x11vnc_${APP_NAME}.log
    ps -p $pid -o cmd= >> /tmp/x11vnc_${APP_NAME}.log
    exit 0
fi

# Start Xvfb for our display
Xvfb :$DISPLAY_NUM -screen 0 1280x1024x16 &
XVFB_PID=$!
echo "Xvfb PID: $XVFB_PID" >> /tmp/x11vnc_${APP_NAME}.log

# Wait for Xvfb to start
sleep 1

# Check if port $VNC_PORT is in use
if netstat -tuln | grep -q :$VNC_PORT; then
    echo "Port $VNC_PORT is already in use, attempting to kill process" >> /tmp/x11vnc_${APP_NAME}.log
    fuser -k $VNC_PORT/tcp
    sleep 1
fi

# Start x11vnc on our display, port $VNC_PORT
x11vnc -display :$DISPLAY_NUM -rfbport $VNC_PORT -rfbauth /home/pi/.vnc/passwd -shared -forever -o /tmp/x11vnc_${APP_NAME}.log &
X11VNC_PID=$!
echo "x11vnc PID: $X11VNC_PID" >> /tmp/x11vnc_${APP_NAME}.log

# Start NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh $VNC_PORT $WS_PORT

# Start the application on our display
DISPLAY=:$DISPLAY_NUM $APP_COMMAND &
APP_PID=$!
echo "$APP_NAME PID: $APP_PID" >> /tmp/x11vnc_${APP_NAME}.log

# Initialize window manager to add titlebars/decorations
/home/pi/sbitx/web/scripts/init_window_manager.sh $DISPLAY_NUM

# Add sleep to let everything start before maximizing the window
sleep 1

# Maximize the window with wmctrl
DISPLAY=:$DISPLAY_NUM wmctrl -r :ACTIVE: -b add,maximized_vert,maximized_horz &
WMCTRL_PID=$!
echo "wmctrl PID: $WMCTRL_PID" >> /tmp/wmctrl_${APP_NAME}.log

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/${APP_NAME}_xvfb.pid
echo "$X11VNC_PID" > /tmp/${APP_NAME}_x11vnc.pid
echo "$APP_PID" > /tmp/${APP_NAME}_app.pid
echo "$WMCTRL_PID" > /tmp/${APP_NAME}_wmctrl.pid

echo "$APP_NAME started"

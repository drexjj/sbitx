#!/bin/bash
# Define the VNC and WebSocket ports for this application
# The webserver will read these values to properly configure the web interface
VNC_PORT=5910
WS_PORT=6090
DISPLAY_NUM=10

# Define the widget label for the web interface
WIDGET_LABEL="Spectrogram Generator   "

# Define the application name and command
APP_NAME="spectrogram"
APP_COMMAND="python3"
APP_ARGS="/home/pi/spectrum_painting/spectrogram-generator.py"

# Stop other apps if needed
# /home/pi/sbitx/web/scripts/stop_wsjtx.sh
# /home/pi/sbitx/web/scripts/stop_fldigi.sh
# /home/pi/sbitx/web/scripts/stop_js8call.sh

# Make sure the scripts are executable
chmod +x /home/pi/sbitx/web/scripts/start_novnc_proxy.sh
chmod +x /home/pi/sbitx/web/scripts/stop_novnc_proxy.sh

# Check if the application is already running
pid=$(pgrep -f "$APP_COMMAND.*$APP_ARGS")
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

# Check if port is in use
if netstat -tuln | grep -q :$VNC_PORT; then
    echo "Port $VNC_PORT is already in use, attempting to kill process" >> /tmp/x11vnc_${APP_NAME}.log
    fuser -k $VNC_PORT/tcp
    sleep 1
fi

# Start x11vnc on our display, port $VNC_PORT
x11vnc -display :$DISPLAY_NUM -rfbport $VNC_PORT -rfbauth /home/pi/.vnc/passwd -shared -forever -o /tmp/x11vnc_${APP_NAME}.log &
X11VNC_PID=$!
echo "x11vnc PID: $X11VNC_PID" >> /tmp/x11vnc_${APP_NAME}.log

# Initialize window manager to add titlebars/decorations
/home/pi/sbitx/web/scripts/init_window_manager.sh $DISPLAY_NUM

# Start the application on our display
export DISPLAY=:$DISPLAY_NUM
export PYTHONUNBUFFERED=1
export GDK_BACKEND=x11
export XDG_RUNTIME_DIR=/run/user/$(id -u)

# Make sure the directory exists
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Start the application with proper environment
cd /home/pi/spectrum_painting
$APP_COMMAND $APP_ARGS --debug > /tmp/${APP_NAME}_app.log 2>&1 &
APP_PID=$!
echo "$APP_NAME PID: $APP_PID" >> /tmp/x11vnc_${APP_NAME}.log

# Maximize the window with wmctrl
DISPLAY=:$DISPLAY_NUM wmctrl -r "${APP_NAME}" -b add,maximized_vert,maximized_horz &
WMCTRL_PID=$!
echo "wmctrl PID: $WMCTRL_PID" >> /tmp/wmctrl_${APP_NAME}.log

# Save PIDs for cleanup
echo "$XVFB_PID" > /tmp/${APP_NAME}_xvfb.pid
echo "$X11VNC_PID" > /tmp/${APP_NAME}_x11vnc.pid
echo "$APP_PID" > /tmp/${APP_NAME}_app.pid
echo "$WMCTRL_PID" > /tmp/${APP_NAME}_wmctrl.pid

echo "$APP_NAME started"

# Start NoVNC proxy for this VNC port
/home/pi/sbitx/web/scripts/start_novnc_proxy.sh $VNC_PORT $WS_PORT

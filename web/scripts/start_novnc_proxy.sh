#!/bin/bash
# Script to start NoVNC proxy for a specific VNC port
# Usage: start_novnc_proxy.sh [VNC_PORT]

# Get the VNC port from command line or use default 5900
VNC_PORT=${1:-5900}
PORT=$((6080 + VNC_PORT - 5900))  # Map 5900->6080, 5901->6081, etc.

# Check if a proxy is already running on this port
if netstat -tuln | grep -q :$PORT; then
    echo "Port $PORT is already in use, proxy may already be running"
    exit 0
fi

# Start the NoVNC proxy
echo "Starting NoVNC proxy: VNC port $VNC_PORT -> WebSocket port $PORT"
/usr/share/novnc/utils/novnc_proxy --vnc localhost:$VNC_PORT --listen $PORT &

# Save the PID for later cleanup
echo $! > /tmp/novnc_proxy_${VNC_PORT}.pid
echo "NoVNC proxy started with PID: $(cat /tmp/novnc_proxy_${VNC_PORT}.pid)"

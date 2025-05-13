#!/bin/bash
# Script to stop NoVNC proxy for a specific VNC port
# Usage: stop_novnc_proxy.sh [VNC_PORT]

# Get the VNC port from command line or use default 5900
VNC_PORT=${1:-5900}

# Check if we have a PID file
if [ -f /tmp/novnc_proxy_${VNC_PORT}.pid ]; then
    PID=$(cat /tmp/novnc_proxy_${VNC_PORT}.pid)
    
    # Check if the process is still running
    if kill -0 $PID 2>/dev/null; then
        echo "Stopping NoVNC proxy for VNC port $VNC_PORT (PID: $PID)"
        kill $PID
        rm /tmp/novnc_proxy_${VNC_PORT}.pid
    else
        echo "NoVNC proxy for VNC port $VNC_PORT is not running (stale PID file)"
        rm /tmp/novnc_proxy_${VNC_PORT}.pid
    fi
else
    echo "No PID file found for VNC port $VNC_PORT"
    
    # Try to find and kill any running novnc_proxy processes for this port
    PORT=$((6080 + VNC_PORT - 5900))
    PIDS=$(ps aux | grep "novnc_proxy.*--listen $PORT" | grep -v grep | awk '{print $2}')
    
    if [ -n "$PIDS" ]; then
        echo "Found NoVNC proxy processes for port $PORT: $PIDS"
        kill $PIDS
    fi
fi

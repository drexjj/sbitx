#!/bin/bash
# Script to stop NoVNC proxy for a specific VNC port
# Usage: stop_novnc_proxy.sh [VNC_PORT] [WS_PORT]

# Get the VNC port from command line or use default 5900
VNC_PORT=${1:-5900}

# Get the WebSocket port from command line or calculate it
if [ -n "$2" ]; then
    # Use the provided WebSocket port
    PORT=$2
else
    # Calculate WebSocket port based on VNC port (legacy behavior)
    PORT=$((6080 + VNC_PORT - 5900))  # Map 5900->6080, 5901->6081, etc.
fi

# First check for new style PID file (with both ports)
if [ -f /tmp/novnc_proxy_${VNC_PORT}_${PORT}.pid ]; then
    PID=$(cat /tmp/novnc_proxy_${VNC_PORT}_${PORT}.pid)
    
    # Check if the process is still running
    if kill -0 $PID 2>/dev/null; then
        echo "Stopping NoVNC proxy for VNC port $VNC_PORT -> WebSocket port $PORT (PID: $PID)"
        kill $PID
        rm /tmp/novnc_proxy_${VNC_PORT}_${PORT}.pid
    else
        echo "NoVNC proxy for VNC port $VNC_PORT -> WebSocket port $PORT is not running (stale PID file)"
        rm /tmp/novnc_proxy_${VNC_PORT}_${PORT}.pid
    fi
# Check for legacy style PID file (with just VNC port)
elif [ -f /tmp/novnc_proxy_${VNC_PORT}.pid ]; then
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
    echo "No PID file found for VNC port $VNC_PORT -> WebSocket port $PORT"
    
    # Try to find and kill any running novnc_proxy processes for this port
    PIDS=$(ps aux | grep "novnc_proxy.*--listen $PORT" | grep -v grep | awk '{print $2}')
    
    if [ -n "$PIDS" ]; then
        echo "Found NoVNC proxy processes for port $PORT: $PIDS"
        kill $PIDS
    fi
fi

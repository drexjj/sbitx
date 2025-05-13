#!/bin/bash
# Stop FLDigi
pid=$(cat /tmp/fldigi_app.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/fldigi_app.pid
fi

# Stop x11vnc
pid=$(cat /tmp/fldigi_x11vnc.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/fldigi_x11vnc.pid
fi

# Stop Xvfb
pid=$(cat /tmp/fldigi_xvfb.pid 2>/dev/null)
if [ -n "$pid" ]; then
    kill $pid 2>/dev/null
    rm /tmp/fldigi_xvfb.pid
fi

echo "FLDigi stopped"
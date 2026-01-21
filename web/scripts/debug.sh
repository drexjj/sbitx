#!/bin/bash
# Enhanced debug script to test execution

# Create a timestamp for consistent logging
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

# Function to write detailed information to a log file
write_debug_info() {
    local LOG_FILE=$1
    echo "======== DEBUG SCRIPT EXECUTION ========" > "$LOG_FILE"
    echo "Timestamp: $TIMESTAMP" >> "$LOG_FILE"
    echo "User: $(whoami)" >> "$LOG_FILE"
    echo "UID: $(id -u)" >> "$LOG_FILE"
    echo "GID: $(id -g)" >> "$LOG_FILE"
    echo "Groups: $(id -G)" >> "$LOG_FILE"
    echo "Current directory: $(pwd)" >> "$LOG_FILE"
    echo "Script path: $0" >> "$LOG_FILE"
    echo "Parent process: $(ps -o user,pid,ppid,cmd -p $PPID)" >> "$LOG_FILE"
    echo "\nEnvironment variables:" >> "$LOG_FILE"
    env | sort >> "$LOG_FILE"
    echo "\nFile permissions:" >> "$LOG_FILE"
    ls -la "$0" >> "$LOG_FILE"
    echo "\nDirectory contents:" >> "$LOG_FILE"
    ls -la "$(dirname "$0")" >> "$LOG_FILE"
    echo "======================================" >> "$LOG_FILE"
    
    # Make the log file readable by everyone
    chmod 666 "$LOG_FILE" 2>/dev/null
}

# Write to multiple locations to ensure at least one works
write_debug_info "/tmp/debug_script.log"
write_debug_info "/home/pi/debug_script.log"
write_debug_info "/home/pi/sbitx/web/scripts/debug_log.txt"

# Create a simple JSON response for web requests
echo '{"status":"success","message":"Debug script executed successfully","timestamp":"'"$TIMESTAMP"'","user":"'"$(whoami)"'"}'

# Also output to stdout for direct viewing
echo "Debug script executed at $TIMESTAMP by $(whoami) in $(pwd)"

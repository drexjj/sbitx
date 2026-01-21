#!/bin/bash
# Simple test script for PHP execution

# Create a log entry to show when the script was run
LOG_FILE="/tmp/app_launchertest_log.txt"

# Function to log messages to both log file and console
log_message() {
    echo "$1" >> "$LOG_FILE"
    echo "$1"
}

# Clear previous log file if it exists
> "$LOG_FILE"

# Log script execution details
log_message "===== TEST SCRIPT EXECUTION LOG ====="
log_message "Date and Time: $(date)"
log_message "User: $(whoami)"
log_message "Working Directory: $(pwd)"

# System information
log_message "\n----- SYSTEM INFORMATION -----"
log_message "Hostname: $(hostname)"
log_message "Kernel: $(uname -r)"
log_message "Uptime: $(uptime)"
log_message "Memory Usage: $(free -h | grep Mem)"
log_message "Disk Space: $(df -h / | tail -1)"

# sBitx information
log_message "\n----- SBITX INFORMATION -----"
log_message "sBitx Version:"
if [ -f /home/pi/sbitx/version.txt ]; then
    cat /home/pi/sbitx/version.txt >> "$LOG_FILE"
    cat /home/pi/sbitx/version.txt
else
    log_message "Version file not found"
fi

# Check if sBitx is running
log_message "\nsBitx Process Status:"
if pgrep -f sbitx > /dev/null; then
    log_message "sBitx is currently running"
    log_message "Process details:"
    ps aux | grep sbitx | grep -v grep >> "$LOG_FILE"
    ps aux | grep sbitx | grep -v grep
else
    log_message "sBitx is not currently running"
fi

# Log script completion
log_message "\n----- SCRIPT COMPLETED -----"
log_message "Finished at: $(date)"
log_message "=============================\n"

# Make sure log file is readable by web server
chmod 644 "$LOG_FILE"

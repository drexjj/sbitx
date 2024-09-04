#!/bin/bash

SOURCE_FILE="/usr/share/applications/sb_launcher.desktop"
DEST_DIR="/etc/xdg/autostart/"

# Check if the file exists
if [ -f "$SOURCE_FILE" ]; then
    # Copy the toolbox file
    sudo cp "$SOURCE_FILE" "$DEST_DIR"

    # Verify
    if [ -f "$DEST_DIR/sb_launcher.desktop" ]; then
        echo "sb_toolbox copied successfully to $DEST_DIR"
    else
        echo "File copy failed."
    fi
else
    echo "Source file does not exist: $SOURCE_FILE"
    fi
    
echo "sBitx-Toolbox will now launch automatically on boot."

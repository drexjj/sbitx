#!/bin/bash

# Check if the script is running as root
if [ "$EUID" -ne 0 ]; then
  echo "This script must be run as root. Re-running with sudo..."
  sudo "$0" "$@"
  exit
fi

# Edit the config.txt file to remove RTC overlay
CONFIG_FILE="/boot/firmware/config.txt"
RTC_OVERLAY="dtoverlay=i2c-rtc-gpio,ds1307,bus=2,i2c_gpio_sda=13,i2c_gpio_scl=6"

echo "Editing $CONFIG_FILE to remove RTC overlay..."
if grep -q "$RTC_OVERLAY" "$CONFIG_FILE"; then
    sed -i "/$RTC_OVERLAY/d" "$CONFIG_FILE"
    echo "RTC overlay removed."
else
    echo "RTC overlay not present."
fi

# Reinstall fake-hwclock and update systemd configuration
echo "Reinstalling fake-hwclock and updating systemd configuration..."
apt -y install fake-hwclock
update-rc.d fake-hwclock defaults

# Uncomment lines in hwclock-set script
HW_CLOCK_SET_FILE="/lib/udev/hwclock-set"
echo "Modifying $HW_CLOCK_SET_FILE..."
sed -i 's/^#if \[ -e \/run\/systemd\/system \] ; then$/if \[ -e \/run\/systemd\/system \] ; then/' "$HW_CLOCK_SET_FILE"
sed -i 's/^#    exit 0$/    exit 0/' "$HW_CLOCK_SET_FILE"
echo "Modification complete."

echo "RTC uninstall script completed. Please reboot for the changes to take effect."

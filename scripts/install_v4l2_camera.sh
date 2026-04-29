#!/bin/bash
set -e

# v4l2 camera stuff
sudo apt-get update
sudo apt-get install -y ros-humble-v4l2-camera
sudo apt-get install -y v4l-utils

# specific to the allied vision camera setup
EXTLINUX_CONF="/boot/extlinux/extlinux.conf"
FDT_LINE="FDT /boot/tegra234-orin-nano-cti-NGX021-AVT-CSI2-2CAM.dtb"

if [ -f "$EXTLINUX_CONF" ]; then
    if grep -Fxq "$FDT_LINE" "$EXTLINUX_CONF"; then
        echo "FDT line already present in $EXTLINUX_CONF"
    else
        echo "$FDT_LINE" | sudo tee -a "$EXTLINUX_CONF" >/dev/null
        echo "Added FDT line to $EXTLINUX_CONF"
    fi
else
    echo "File not found: $EXTLINUX_CONF"
fi



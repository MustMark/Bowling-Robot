#!/bin/bash
# clear_camera.sh

DEVICE="/dev/video0"

echo "Checking camera device: $DEVICE"
if [ ! -e "$DEVICE" ]; then
    echo "Camera device not found ($DEVICE)"
    exit 1
fi

PIDS=$(sudo fuser "$DEVICE" 2>/dev/null)

if [ -z "$PIDS" ]; then
    echo "Camera is free. Running cv2.destroyAllWindows()..."
    python3 -c "import cv2; cv2.destroyAllWindows(); print('All OpenCV windows closed')"
else
    echo "Camera is currently in use by process(es): $PIDS"
    read -p "Do you want to force close these processes? (y/n): " answer
    if [ "$answer" == "y" ] || [ "$answer" == "Y" ] || [ "$answer" == "yes" ] || [ "$answer" == "Yes" ]; then
        sudo kill -9 $PIDS
        echo "Killed processes using the camera."
        sleep 1
        python3 -c "import cv2; cv2.destroyAllWindows(); print('All OpenCV windows closed')"
    else
        echo "Camera still in use."
    fi
fi

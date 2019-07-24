# Video mapping
export MEMMAP_PATH="/tmp" # Memmap video variables location 
export CAM_PORTS_PATH="$PWD/cam_ports.yaml" # Absolute path to camera ports file
export VIDEO_HEIGHT=360 # Video camera streaming height
export VIDEO_WIDTH=640 # Video camera streaming width

# Data capture
export DATA_CAPTURE=1 # Enable(1)/Disable(0) data capture in launch
export MIN_USB_SPACE=3 # [%] minimum percentage of usb to write images
export DATA_CAPTURE_FPS=15 # Desired frame rate for data capture
export IMG_QUALITY=80 # (0-100) Quality of images recorded

# General Enviorement variables
export LOCAL_RUN=1 # Enable(1)/Disable(0) local run
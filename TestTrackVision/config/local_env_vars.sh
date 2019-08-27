# Video mapping
export MEMMAP_PATH="/tmp" # Memmap video variables location 
export CAM_PORTS_PATH="$PWD/cam_ports.yaml" # Absolute path to camera ports file
export VIDEO_HEIGHT=360 # Video camera streaming height
export VIDEO_WIDTH=640 # Video camera streaming width
export CALIBRATION_PATH="${PWD%/*/*}/PostScripts/Calibration_Utils/Gallery"

# Data capture
export DATA_CAPTURE=0 # Enable(1)/Disable(0) data capture in launch
export MIN_USB_SPACE=3 # [%] minimum percentage of usb to write images
export IMG_QUALITY=80 # (0-100) Quality of images recorded

# General Enviorement variables
export LOCAL_RUN=1 # Enable(1)/Disable(0) local run

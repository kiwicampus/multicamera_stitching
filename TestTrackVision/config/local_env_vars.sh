# Video mapping variables
export MEMMAP_PATH="/tmp" # Memmap video variables location 
export CAM_PORTS_PATH="$PWD/cam_ports.yaml" # Absolute path to camera ports file
export VIDEO_HEIGHT=360 # Video camera streaming height
export VIDEO_WIDTH=640 # Video camera streaming width
export CALIBRATION_PATH="${PWD%/*/*}/PostScripts/Calibration_Utils/Gallery"

# Data capture Enviorement variables
export DATA_CAPTURE=1 
    # Enable(1): No data capture node launch
    # Disable(0): Data capture in launch
export DATA_CAPTURE_MIN_USB_SPACE=3 # [%] minimum percentage of usb to write images
export DATA_CAPTURE_IMG_QUALITY=80 # (0-100) Quality of images recorded

# General Enviorement variables
export LOCAL_RUN=1
    # Disable (0): No Local GUI
    # Enable  (1): Show Data from Cameras
    # Enable  (2): Load Data from folder
# For LOCAL_RUN==2
export LOCAL_DATA_PATH="/home/kiwivision/Downloads/data_capture-08-28-19"
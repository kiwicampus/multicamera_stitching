# Video mapping
export MEMMAP_PATH="/tmp" # Memmap video variables location 
export CAM_PORTS_PATH="configs/cam_ports.yaml" # Absolute path to camera ports file
export VIDEO_HEIGHT=360 # Video camera streaming height
export VIDEO_WIDTH=640 # Video camera streaming width

# Data capture
export MIN_USB_SPACE=3 # [%] minimum percentage of usb to write images
export DATA_CAPTURE_FPS=15 # Desired frame rate for data capture
export DATA_CAPTURE=1 # Enable(1)/Disable(0) data capture in launch
export USB_MOUNT=~/local_sim_data/ # absolute path to mount device if IS_SIM=1

# General Enviorement variables
export LOCAL_RUN=0 # Enable(1)/Disable(0) local run
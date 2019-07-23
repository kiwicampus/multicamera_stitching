# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision &Ai Team

Supported Camera Resolutions: 
    Aspect ratio 4:3  -> (240, 320), (480, 640), (600, 800)
    Aspect ratio 16:9 -> (180, 320), (360, 640), (720, 1280), (1080, 1920)
"""

# =============================================================================
import numpy as np
import time
import yaml
import os

import subprocess
import rospy
import cv2
import io
import re

from threading import Thread, Event

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from video_mapping.srv import GetMoreCameraStatus
from video_mapping.srv import GetMoreCameraStatusResponse

# =============================================================================
def read_cam_ports(file_path):
    """ Reads the camera labels and port from file
    Args:
        file_path: `string` absolute path to camera labels and port file
    Returns:
        _: `dictionary` key: camera labels, values: camera ports
    """
    abs_path = os.path.join(os.path.dirname(__file__), file_path)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        default_ports = {'CAM1': '1.0.0'}
        with io.open(abs_path, 'w', encoding='utf8') as outfile:
            yaml.dump(default_ports, outfile, default_flow_style=False, allow_unicode=True)
        return default_ports

def find_cameras(debugger):
    """ Finds the camera numbers and ports that correspond to real cameras 
        Note: (devices may be repeated)
    Args:
    Returns:
        usb_ports_cameras" `list` list with video devices ports and numbers
    """

    # Finds and lists video devices numbers
    cams = find_video_devices()

    # Finds and lists video devices ports
    avail_ports = find_usb_ports(debugger, cams)

    # Zip video devices ports and numbers
    usb_ports_cameras = dict()
    for port, cam in zip(avail_ports, cams):
        if not port in usb_ports_cameras:
            usb_ports_cameras[port] = cam

    # list with video devices ports and numbers
    return usb_ports_cameras

def find_video_devices():
    """ Finds and lists video devices numbers
    Args:
    Returns:
        cams_list: `list` with video camera devices numbers
    """

    # Check for video devices
    p = re.compile(r".+video(?P<video>\d+)$", re.I)
    devices = subprocess.check_output("ls -l /dev", shell=True).decode('utf-8')
    avail_cameras = []

    for device in devices.split('\n'):
        if device:
            info = p.match(device)
            if info:
                dinfo = info.groupdict()
                avail_cameras.append(dinfo["video"])
    cams_list = list(sorted(map(int,avail_cameras)))

    return cams_list
 
def find_usb_ports(debugger, cameras):
    """ Finds and lists video devices ports
    Args:
        cameras: `list` with video camera devices numbers
    Returns:
        avail_ports: `list` with video camera devices ports
    """
    
    # find usb port given a video number device
    avail_ports = []
    p = re.compile(r"\d-(?P<video>[0-9.]+).+", re.I)

    for cam in cameras:
        try:
        # List of physical ports used in /dev/video# (some devices maybe represent same thing)
            path = subprocess.check_output("udevadm info --query=path --name=/dev/video" + str(cam), shell=True).decode('utf-8')

        except Exception as e:
            debugger(DEBUG_LEVEL_0, "----- ERROR READING VIDEO DEVICE ----- (Error: {})".format(e), log_type = 'err')
            avail_ports.append('None-{}'.format(cam))
            continue

        debugger(DEBUG_LEVEL_0, path.strip('\n'))
        path = path[0:path.find("video4linux")].split('/')[-2] #get last item where the port is explicit
        info = p.match(path) #get actual address
        if info:
            dinfo = info.groupdict()
            avail_ports.append(dinfo["video"])

    return avail_ports

# =============================================================================
class CameraHandler(Thread, Debugger):

    def __init__(self, camera_number, height, width, camera_id):
        """ Initializes camera Handler thread 
        Args:
            camera_number: `int` camera port number
            height: `int` video camera height
            width: `int` video camera width
            camera_id: `int` camera identifier
        Returns:
        """

        # start legacy component
        super(CameraHandler, self).__init__() 

        self.camera_number = camera_number # Camera port number
        self.video_device = "/dev/video" + str(camera_number) # Camera video device
        self.height = height # Video height
        self.width = width # Video width
        self.camera_id = camera_id # Camera label

        self.image = np.zeros((self.height, self.width, 3), dtype=np.uint8) # Camera Image
        self.grabbed = False # Camera status
        self.disconnected=False

        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.daemon = True
        self.video_handler = None

        # Start camera port
        self.init_video_handler() 

    def init_video_handler(self):
        """ InitializeS video handler
        Args:
        Returns:
        """

        # Open camera port
        self.video_handler = cv2.VideoCapture(self.video_device)

        # opens successfully the camera
        if self.video_handler.isOpened(): 

            # Set desired size to camera handler
            # Some OpenCV versions requirers assing the size twice -Dont know why
            self.set_video_size(); self.set_video_size()

            try: # Grab frame from camera and assing properties                
                self.grabbed, self.image = self.video_handler.read()

                if self.grabbed:
                    self.debugger(DEBUG_LEVEL_0, "{}:{} captured first image".format(
                        self.camera_id, self.video_device), log_type="info")

                    # If camera was disconnected
                    if self.disconnected:
                        self.disconnected=False 
                        self.debugger(DEBUG_LEVEL_0, "{}:{} has been reconnected".format(
                            self.camera_id, self.video_device), log_type="info")

                    # Print info if video size can not be setted
                    if self.image.shape[0]!=self.height or self.image.shape[1]!=self.width:
                        self.image=cv2.resize(self.image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
                        self.debugger(DEBUG_LEVEL_0, "{}:{} can't set video size, resize will be performed".format(
                            self.camera_id, self.video_device), log_type="warn")
                else:
                    self.debugger(DEBUG_LEVEL_0, "{}:{} did not capture first image, possibly not space left on device".format(
                        self.camera_id, self.video_device), log_type="warn")
                    self.set_error_image("{} NO SPACE ON DEVICE".format(self.camera_id))

            # If not possible open camera port print exception 
            except Exception as e:
                self.debugger(DEBUG_LEVEL_0,
                    "Something error ocurred reading frames from camera {} (device {}) -> {}".format(
                        self.camera_id, self.video_device, e), log_type = 'err')
                self.grabbed = False
                self.set_error_image("{} READING ERROR".format(self.camera_id))
        else:
            if not self.disconnected:
                self.debugger(DEBUG_LEVEL_0, "{}:{} not recognized".format(
                    self.camera_id, self.video_device), log_type="warn")
            self.set_error_image("{} {}".format(self.camera_id, "NO RECOGNIZED" if not self.disconnected else "DISCONNECTED"))
            self.video_handler = None
    
    def set_video_size(self):
        """ Assigns video handler's size
        Args:
        Returns:
        """
        self.video_handler.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.video_handler.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)

    def set_error_image(self, error_msg):
        """ Set in camera thread image a default error image
        Args:
            error_msg: `string` error message to print on image
        Returns:
        """
        self.image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        font_size = 0.0015625*self.width + 0.5 #empirical formula so that: when w=6400 -> font=1.5 and w=1920 -> font=3.5
        cv2.putText(self.image, error_msg, (self.width//8, self.height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, font_size, (255,255,255), 4)

    def run(self):
        tries_limit=0
        if self.video_handler is not None:
            while self.run_event.is_set():
                if self.grabbed: 
                    try:
                        self.grabbed, image = self.video_handler.read()
                        self.image=cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_LINEAR
                            ) if image.shape[0]!=self.height or image.shape[1]!=self.width else image
                            
                    except Exception as e: # Error reading image
                        self.set_error_image("{} ERROR WHILE READING".format(self.camera_id))
                        self.debugger(DEBUG_LEVEL_0, "Camera: {} error while reading".format(
                            self.camera_id), log_type = 'err')
                        self.disconnected = True

                    if not self.grabbed: # If camera was disconnected
                        self.debugger(DEBUG_LEVEL_0, "Camera: {} HOT Unplugged!".format(
                            self.camera_id), log_type = 'err')
                        self.video_handler.release()
                        self.video_handler = None
                        self.disconnected = True
                        
                elif self.disconnected:
                    self.init_video_handler()
                    time.sleep(1) # wait as if a camera is disconnected
                    # find_cameras(self.debugger)
                    tries_limit+=1
                    if tries_limit>120: # in [seconds]
                        self.debugger(DEBUG_LEVEL_0, "Camera: {} has reached the limit of reconnecting trials".format(
                            self.camera_id), log_type = 'warn')
                        self.set_error_image("{} LOST".format(self.camera_id))
                        break
                else:
                    time.sleep(0.10) # wait as if a frame were read

            if self.video_handler is not None:
                self.video_handler.release()
                self.video_handler = None

class CamerasSupervisorBase(Debugger):

    def __init__(self, CameraClass=CameraHandler):

        # ---------------------------------------------------------------------
        # start legacy components
        super(CamerasSupervisorBase, self).__init__()

        # Set cameras labels
        self.cameras_labels = read_cam_ports(os.environ.get("CAM_PORTS_PATH", 
            "configs/cam_ports.yaml"))

        # Get video/image dimensions
        self.video_height = int(os.environ.get("VIDEO_HEIGHT", 360))
        self.video_width = int(os.environ.get("VIDEO_WIDTH", 640))

        # Get available connected cameras
        self.usb_ports_cameras = find_cameras(self.debugger)

        # Print some info
        self.debugger(DEBUG_LEVEL_0, "Video device numbers detected: {}".format(
            len(self.usb_ports_cameras)), log_type="warn" if not len(self.usb_ports_cameras) else "info")
        
        # ---------------------------------------------------------------------
        # Set process variables
        self.video_numbers = [self.usb_ports_cameras[str(port)] if port in self.usb_ports_cameras else None for port in self.cameras_labels.values() ]
        self.num_cameras = len(self.cameras_labels)

        # ---------------------------------------------------------------------
        # Initializes camera handler threads objects
        
        self.camera_handlers = [CameraClass(
            video, self.video_height, self.video_width, camera_id) 
            for video, camera_id in zip(self.video_numbers, self.cameras_labels)]

        # Check cameras status
        self.cameras_status = [cam_hand.grabbed for cam_hand in self.camera_handlers]

        # Print some info
        self.debugger(DEBUG_LEVEL_0, "Shape of video mapping array: {}X{}X{}".format(
            self.video_height, self.video_width, 3*self.cameras_status.count(True)))

        # TODO: Create services to report cameras status
        # rospy.Service('video_mapping/get_cameras_status_verbose', GetMoreCameraStatus, self.get_cameras_status_verbose)

    def get_cameras_status_verbose(self, data):
        self.cameras_status, debug_str = self.check_cameras(self.camera_handlers, self.cameras_labels, stdout=False)
        return GetMoreCameraStatusResponse(debug_str)

class CamerasSupervisor(CamerasSupervisorBase, Debugger):

    def __init__(self, *args, **kwargs):

        # Create handlers for each specified camera
        super(CamerasSupervisor, self).__init__(*args, **kwargs)

        # Start video capture of handlers
        map(lambda o: o.start(), self.camera_handlers)

# =============================================================================